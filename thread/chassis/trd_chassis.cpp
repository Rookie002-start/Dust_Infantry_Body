/**
 * @file trd_chassis.cpp
 * @author Rookie002-start
 * @brief 底盘控制线程 — 1ms 固定周期：指令 → 坐标变换 → 麦轮逆解 → 速度环 → 组帧
 * @version 0.4
 * @date 2026-09-29
 *
 * ## 坐标系（右手系，z 轴向上）
 *
 *       +x (前)
 *        ↑
 *        |
 *        O────→ +y (左)
 *
 * 自转 ω 绕 z 轴，逆时针为正。
 *
 * 上板指令的 y 轴正向是"车体右移"（遥控横移量的自然方向），与本坐标系相反，
 * 因此在 ReadCommand 里读进来时就取反一次（见该处注释），之后
 * 旋转 → 逆解 → 速度环 整条链路都统一在" +y = 左 "下计算。
 *
 * ## 数据来源与所有权
 *
 * | 数据 | 通道 / 段 | 生产者 | 本线程的角色 |
 * |------|-----------|--------|--------------|
 * | 上板指令 | zbus `pub_from_head` | thread/inter_rx | 轮询读（状态通道无订阅者） |
 * | 云台相对角 | zbus `pub_gimbal_to` | 云台线程（尚未接入） | 轮询读，缺数据按 0 处理 |
 * | 电机反馈 | `.can_rx1` 段 | 本线程（总线所有者） | 只注册 CAN_RX_HANDLER |
 * | 控制电流 | k_msgq `user_can1_msgq` | 本线程 | 入队后本线程外发 |
 * | 波形输出 | VOFA+ JustFloat（别名 `vofa-uart`） | 本线程 | 每拍发被测轮 4 通道 |
 *
 * 总线所有权：user-can1 由本线程持有并初始化（Init + 收帧分发入口 + 外发），
 * 与云台线程（can3）、inter_bus（can2）的约定一致。thread/can 只是单总线
 * 收发测试线程，不参与本总线（Kconfig 里 TRD_CAN 与 TRD_CHASSIS 互斥）。
 *
 * ## 控制流
 *
 *     ReadCommand()       上板指令 + online 判定（离线 → 停车）
 *         ↓
 *     ReadGimbalYaw()     云台相对底盘偏航角
 *         ↓
 *     UpdateTarget()      云台系 → 底盘系旋转 + 麦轮逆解 + 限幅
 *         ↓
 *     ControlCalculate()  单轮速度环 PID（ω → 目标电流）
 *         ↓
 *     FramePublish()      4 × int16 组帧 → CAN 发送队列
 *         ↓
 *     FlushTx()           队列 → user_can1.Send()（本线程是总线所有者）
 *         ↓
 *     VofaPublish()       被测轮 4 通道 → VOFA+（JustFloat，走 vofa-uart）
 *
 * ## 单速度环结构（每个电机一组）
 *
 *     目标轮速 ω_ref ──→ ⊖ ──→ [速度 PID] ──→ i_ref [A] ──→ ×kCurrentScale ──→ CAN 电流
 *                       ↑
 *                    实测 ω
 *
 *   回路只有一环：反馈是输出轴轮速 ω（模块已按减速比折算，与目标同单位），
 *   输出直接是"目标电流"（A），组帧时按 kCurrentScale 换成 C620 的原始值；
 *   中间不再有内环/力矩量，电流限幅即 kCurrentMax。
 *
 * ## 整定（单速度环）
 *
 *   ① 看 VOFA+：ch0(ω_ref)/ch1(ω_meas) 调 kSpeedKp——边沿够快、又不过冲振铃为准，
 *      再加一点 kSpeedKi 消静差（高频抖动/啸叫 = kp 过头）；
 *   ② 同时看 ch2(i_ref)：被顶到 ±kCurrentMax 说明增益给得太猛（或负载太重），先降 kp；
 *   ③ 整定完把数字抄回下面的 kSpeedKp/kSpeedKi，重编译烧录。
 *
 * ## 待实测参数（当前为占位值，实车测量后锁定）
 *
 * | 参数 | 当前值 | 说明 |
 * |------|--------|------|
 * | `kWheelRadius` | 0.075 m | 轮半径（轮径 0.15 m） |
 * | `kChassisR` | 0.19 m | 轮心到车体中心的等效自转半径 |
 * | `kMotorPhaseResist` | 0.15 Ω | M3508 相电阻（K1 = R/Kt² 由它算出），待实测替换 |
 * | `kWheelMix[][]` | ±√2/2 | 麦轮混控矩阵（含安装极性，方向不对只改这张表） |
 * | 速度环 | kp=1.0 / ki=0.10 | 输出 = 目标电流 A，需按实车整定 |
 *
 * ## 上电自校准（k3）
 *
 * 线程起来后先跑一次 `CalibrateK3()`：等功率计第一帧 → 平均 500 ms → 除以 4 台，
 * 得到模型里的静态损耗 k3。这是模型里唯一能直接实测的参数，猜错会被 K1/K2 吸收。
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Chassis"

#include "from_head.hpp"
#include "gimbal_to.hpp"
#include "inter_cmd.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "trd_chassis.hpp"
#include "can.hpp"
#include "vofa.hpp"
#include "power_ctrl.hpp"

#include <algorithm>
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(chassis, LOG_LEVEL_INF);

namespace {

/// √2/2：麦轮驱动方向与车体轴成 45°，车体速度投影到轮上有这个归一化系数
constexpr float kSqrt2_2 = 0.70710678f;

/// 指令归一化限幅（上板的异常值不允许放大成危险速度）
float ClampUnit(float v) { return std::clamp(v, -1.0f, 1.0f); }

} // namespace

namespace thread::chassis {

using namespace instance::chassis;

static Thread<2048> thread_ {};
static Can user_can1 {};                 // user-can1 总线对象（本线程私有，本线程是总线所有者）

// 功率控制器
static alg::power_ctrl::PowerCtrl<kMotorCount> ChassisPwrCtrl {}; 

// ==================== 机械 ====================
static constexpr float kWheelRadius  = 0.075f;            // 轮半径 m
static constexpr float kChassisR     = 0.19f;           // 轮心到车体中心等效自转半径 m
static constexpr float kGearboxRatio = 3591.f / 187.f;   // M3508 + C620 减速比

// ==================== 电气（C620 + M3508） ====================
static constexpr float kTorqueK      = 0.3f;                     // 转矩常数 N·m/A（传给电机模块换算力矩，控制环不用）
// 速度环输出的电流上限（电调量程仍是 ±20 A ↔ ±16384）：组帧前按 kCurrentMax 限一次，
// 越界会整数回绕；kCurrentScale 把 A 换成 C620 的原始值。
static constexpr float kCurrentMax   = 20.0f;
static constexpr float kCurrentScale = 16384.0f / kCurrentMax;   // A → 原始值

// ==================== 功率限制（待实测锁定） ====================
#if CONFIG_USE_POWERMETER
static constexpr bool  kRlsEnable = true;                  // 有功率计 → 在线辨识
#else
static constexpr bool  kRlsEnable = false;                 // 无功率计 → 固定 K1/K2
#endif
static constexpr float kPowerBudget       = 45.0f;         // 总功率预算 W
static constexpr float kMotorPhaseResist  = 0.15f;         // M3508 相电阻 Ω
static constexpr float kPwrK1Init         = kMotorPhaseResist / (kTorqueK * kTorqueK);  // ≈ 1.67
static constexpr float kPwrK2Init         = 8.0e-03f;
static constexpr float kPwrK1Min = 1.0f;
static constexpr float kPwrK1Max = 2.5f;
static constexpr float kPwrK2Min = 0.007f;
static constexpr float kPwrK2Max = 0.009f;
static constexpr float kPwrK3             = 0.75f;
static constexpr float kPwrEta            = 1.0f;
static constexpr float kPwrErrUpper       = 8.0f;          // 全按需求分配（加速/打滑时）
static constexpr float kPwrErrLower       = 1.0f;          // 全按功率分配（稳态跟踪时）
static constexpr float kPwrMinBudgetShare = 0.05f;         // 每台保底份额，防某轮被分到 0 预算后掉出驱动
// 回馈（制动倒灌）功率上限 W
static constexpr float kPwrRegenMaxW      = 120.0f;
// 限幅回写速度环积分的强度：1.0 = 单拍把"被削掉的那部分"从积分里扣干净（最激进）
static constexpr float kPwrAntiWindupGain = 1.0f;
// RLS 每收到一次**新测量帧**才迭代一次，遗忘时间常数 ≈ 1/((1-λ)·帧率)；
// 功率计 500Hz、λ=0.998 → 约 1s 记忆
static constexpr float kPwrRlsLambda      = 0.998f;
#if CONFIG_USE_POWERMETER
static constexpr float kPwrMeterMinBusVolt = 1.0f;         // 功率计在线判据 V
static constexpr uint32_t kPwrMeterTimeoutMs = 30;         // 超过这么久没新帧 → 掉线（500Hz 下约 15 帧）
static constexpr uint8_t  kPwrMeterResumeFrames = 10;      // 连续这么多帧正常才算恢复（滞回）
static constexpr uint32_t kK3CalibWaitMs  = 1000;          // k3 自校准：等功率计第一帧的上限
static constexpr uint32_t kK3CalibMs      = 500;           // k3 自校准：平均时长
static constexpr uint8_t  kK3CalibTries   = 2;             // k3 自校准重试次数
// 整机静态功率的合理区间：功率计上电后可能先报几帧 0（还没出有效值），
// 直接采信会把 k3 记成 0、模型丢掉静态项，所以落在区间外就重试/放弃
static constexpr float    kK3CalibMinW    = 0.5f;
static constexpr float    kK3CalibMaxW    = 50.0f;
#endif

static constexpr float kSpeedKp = 1.0f;              // 速度环 kp：轮速误差 → 目标电流 A
static constexpr float kSpeedKi = 0.10f;             // 速度环 ki

// VOFA+ 发哪一路（0~3 → CAN 0x201~0x204）。想看别的轮子改这个常量即可。
static constexpr uint8_t kVofaWheel = 0;

// ==================== 指令限幅与周期 ====================

static constexpr float    kMaxWheelOmega    = 20.0f;     // 单轮输出轴角速度上限 rad/s（安全限）。
static constexpr float    kMaxMoveVelocity  = kMaxWheelOmega * kWheelRadius / kSqrt2_2;
static constexpr float    kMaxRotationOmega = 8.0f;      // 自转角速度上限 rad/s
static constexpr uint32_t kPeriodMs         = 1;         // 控制周期
static constexpr uint16_t kChassisTxId      = 0x200;     // 底盘控制帧 ID（4 × int16 电流）

static constexpr float kWheelMix[kMotorCount][3] = {
    { +kSqrt2_2, -kSqrt2_2, -kSqrt2_2 },   // 电机1（0x201）
    { +kSqrt2_2, +kSqrt2_2, -kSqrt2_2 },   // 电机2（0x202）
    { +kSqrt2_2, -kSqrt2_2, +kSqrt2_2 },   // 电机3（0x203）
    { +kSqrt2_2, +kSqrt2_2, +kSqrt2_2 },   // 电机4（0x204）
};

/// 电机安装极性（正电流 → 车体正方向）：实测后锁定
static constexpr int8_t kMotorPolarity[kMotorCount] = { -1, -1, +1, +1 };

// ==================== 本线程私有状态 ====================
// 只有 Task 读写，不需要锁；不要与其他线程共享这些量
static bool chassis_stop = false;
static float g_vx        = 0.0f;                // 底盘系前后速度 m/s
static float g_vy        = 0.0f;                // 底盘系左右速度 m/s
static float g_vw        = 0.0f;                // 自转角速度 rad/s
static bool  g_cmdOnline = false;               // 上板指令是否有效
static float g_yawGimbal = 0.0f;                // 云台相对底盘偏航角 rad
static bool  g_yawValid  = false;               // 是否收到过有效云台角
static float g_wheelTarget[kMotorCount]  {};    // 各轮目标角速度 rad/s
static float g_wheelCurrent[kMotorCount] {};    // 各轮速度环输出 = 目标电流 A（功率限制插入点）
static uint32_t g_txDrop = 0;                   // 发送队列丢帧计数（诊断用）

// 诊断 / VOFA+ 波形用的同帧快照（ControlCalculate 里从 ReadAll() 一次取齐）
static float g_wheelOmegaMeas[kMotorCount]   {}; // 实测轮速 rad/s（输出轴）
static float g_wheelCurrentMeas[kMotorCount] {}; // 实测电流 A

static constexpr uint32_t kTxDropLogPeriod = 1000;   // 累计丢帧达到该值上报一次
static constexpr uint8_t  kTxPerTickMax    = 4;      // 每拍最多外发帧数（1ms 内 4 × 8B ≈ 0.5ms 总线时间）
static uint32_t g_txFail = 0;                        // CAN 发送失败计数（bus-off / 邮箱满时诊断用）

#if CONFIG_USE_POWERMETER
static uint32_t g_pwrFrames   = 0;                   // 上一拍读到的功率计帧号（判新样本）
static uint8_t  g_pwrOkStreak = 0;                   // 连续正常帧数（恢复滞回用）
#endif


/**
 * @brief 上板指令：底盘系线速度 / 自转 + 在线判定
 *
 * 状态通道 `pub_from_head` 由 thread/inter_rx 周期发布，且定义为
 * ZBUS_OBSERVERS_EMPTY（无订阅者），因此这里必须用 zbus_chan_read 轮询：
 * 既不占 subscriber，也不会因为多线程共用 subscriber 互相抢通知。
 *
 * 上板离线（online=false）、或遥控链路失效（CommState.flags 没有 LinkOk 位）时
 * 立即停车并清 PID 积分：底盘不允许使用陈旧指令。
 */
static void ReadCommand()
{
    static topic::from_head::Message msg {};
    static bool online_prev = false;

    if (zbus_chan_read(&pub_from_head, &msg, K_NO_WAIT) != 0) {
        // 读忙（发布者正在更新）：沿用上一轮指令，不要清零
        return;
    }

    // 只看 CAN 帧有没有到是不够的：上板数据源（遥控）掉线时帧照样每 2ms 到，
    // 只是里面是"全 0 + 无 LinkOk"的失效指令。两种情况都必须停车。
    const bool cmd_ok = msg.online && inter_cmd::CommLinkOk(msg.comm);

    if (!cmd_ok) {
        g_vx = 0.0f;
        g_vy = 0.0f;
        g_vw = 0.0f;
        for (auto &pid : chassis_motor_omega_pid) {
            pid.SetIntegralError(0.0f);          // 防止恢复在线时积分残留冲一下
        }
    } else {
        g_vx = ClampUnit(msg.comm.chassis_vx) * kMaxMoveVelocity;
        g_vy = ClampUnit(msg.comm.chassis_vy) * kMaxMoveVelocity;

        switch (static_cast<inter_cmd::SpinMode>(msg.comm.chassis_spin)) {
        case inter_cmd::SpinMode::Spin:
            g_vw = kMaxRotationOmega;            // 小陀螺：满速自转
            break;
        case inter_cmd::SpinMode::Stop:
            g_vw = 0.0f;                         // 上板要求停转
            chassis_stop = true;
            break;
        default:
            g_vw = ClampUnit(msg.comm.chassis_rot) * kMaxRotationOmega;
            chassis_stop = false;
            break;
        }
    }

    if (cmd_ok != online_prev) {
        if (!msg.online) {
            LOG_INF("command link OFFLINE (head lost) -> stop");
        } else if (!inter_cmd::CommLinkOk(msg.comm)) {
            LOG_INF("command link OFFLINE (remote failsafe) -> stop");
        } else {
            LOG_INF("command link online");
        }
        online_prev = cmd_ok;
    }
    g_cmdOnline = cmd_ok;
}

/**
 * @brief 云台相对底盘偏航角（chassis-frame 变换的输入）
 *
 * 云台线程（生产者）尚未接入本板：
 *   · 从未收到有效角 → 按 0 处理，即"云台朝前"，指令不做旋转，车仍能正常开；
 *   · 收到过之后掉线 → 沿用最后一次有效角（云台机械上仍停在那里，跳回 0 更危险）。
 */
static void ReadGimbalYaw()
{
    static topic::gimbal_to::Message msg {};

    if (zbus_chan_read(&pub_gimbal_to, &msg, K_NO_WAIT) != 0 || !msg.online) {
        return;
    }

    g_yawGimbal = msg.yaw_rad;
    if (!g_yawValid) {
        g_yawValid = true;
        LOG_INF("gimbal yaw source online");
    }
}

/**
 * @brief 云台系 → 底盘系旋转 + 麦轮逆解
 *
 * 上板给的是"云台坐标系"下的平移指令，底盘要按云台相对角 θ 转到车体系：
 *
 *     [vx_c]   [ cosθ  -sinθ ] [vx_g]
 *     [vy_c] = [ sinθ   cosθ ] [vy_g]
 */
static void UpdateTarget()
{
    if (!g_cmdOnline || chassis_stop) {
        // 停机态：目标直接归零，不做无意义的逆解（速度环仍工作 → 主动刹住轮子）
        for (auto &target : g_wheelTarget) {
            target = 0.0f;
        }
        return;
    }

    const float c  = cosf(g_yawGimbal);
    const float s  = sinf(g_yawGimbal);
    const float vx = c * g_vx - s * g_vy;
    const float vy = s * g_vx + c * g_vy;

    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        const float v_wheel = kWheelMix[i][0] * vx
                            + kWheelMix[i][1] * vy
                            + kWheelMix[i][2] * (kChassisR * g_vw);

        g_wheelTarget[i] = std::clamp(kMotorPolarity[i] * v_wheel / kWheelRadius,
                                      -kMaxWheelOmega, kMaxWheelOmega);

    }
}

/**
 * @brief 单轮速度环 PID：轮速 → 目标电流
 *
 * 反馈统一取 ReadAll() 快照（模块内 seqlock 保护）：ω 与实测电流必须来自同一帧，
 * 分开调用两个 getter 有可能取到不同帧的数据。ω 是输出轴 rad/s（模块已按
 * 减速比折算），与目标同单位，不需要再乘/除减速比。
 */
static void ControlCalculate()
{
    if (chassis_stop)
    {
        // 停机必须连"真正发出去的电流"一起清零：FramePublish() 组帧用的是
        // g_wheelCurrent[]，不清零就 return 的话，电流会保持停机前最后一拍的
        // 值（例如满量程 20 A），遥控器打 Stop 也停不下来。
        for (uint8_t i = 0; i < kMotorCount; ++i)
        {
            // 实测快照同样要刷新：PowerAllocate() 拿它喂功率控制器，留着停机前的
            // 陈旧 τ/ω 会让 RLS 把"已经停机、电流却还是 8 A"的假样本吃进去
            const auto snap = chassis_motor[i].ReadAll();
            g_wheelOmegaMeas[i]   = snap.omega;
            g_wheelCurrentMeas[i] = snap.current;

            g_wheelCurrent[i] = 0.0f;
        }
        for (auto &pid : chassis_motor_omega_pid) {
            pid.SetIntegralError(0.0f);      // 解除停机时不要带着停机前的积分冲一下
        }
        return;
    }
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        const auto snap = chassis_motor[i].ReadAll();

        // ω 与 i 来自同一帧快照，波形上两条曲线才对得上
        g_wheelOmegaMeas[i]   = snap.omega;
        g_wheelCurrentMeas[i] = snap.current;

        // 速度环：目标角速度 → 目标电流（电调收的就是电流，输出限幅在 kCurrentMax）
        g_wheelCurrent[i] = chassis_motor_omega_pid[i].Calc(g_wheelTarget[i], snap.omega);

        // g_wheelCurrent[i] = 0;   //测试用
    }
}

/**
 * @brief 把本拍数据喂给功率控制器：实测 τ/ω + PID 目标电流
 *
 * 只喂不预测，台架测试模式也要用（见"板内台阶测试"）。
 */
static void FeedPowerModel()
{
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        const float tau = g_wheelCurrentMeas[i] * kTorqueK;   // 实测电流 → 转矩

        ChassisPwrCtrl.SetMotorData(i, tau, g_wheelOmegaMeas[i],
                                    g_wheelTarget[i] - g_wheelOmegaMeas[i]);
        ChassisPwrCtrl.SetTarget(i, g_wheelCurrent[i]);       // PID 目标电流
    }
}

/**
 * @brief 功率预测 + 分配：把速度环的目标电流限到功率预算内
 *
 * 喂数据（实测 τ/ω + PID 目标电流）→ 预测(+RLS) → 分配 → 写回 g_wheelCurrent[]，
 * 必须在 ControlCalculate() 之后、FramePublish() 之前调用。τ/ω 取同帧实测值。
 *
 * 功率计只在**新帧**上迭代 RLS，掉线（数据年龄超时）后冻结 K1/K2 走开环预测；
 * 末尾把"被削掉的那部分电流"折回速度环积分，防止限流期间积分饱和。
 */
static void PowerAllocate()
{
    FeedPowerModel();

    // 功率计快照 + 新鲜度 / 掉线判定
    bool meterOk = false;
#if CONFIG_USE_POWERMETER
    {
        const auto pm = ChassisPwrMeter.ReadAll();

        // 有没有新帧：RLS 只在新样本上迭代。功率计 500Hz、控制环 1kHz，
        // 若每拍都喂，会让同一条测量值被迭代两遍（x 在变、y 没变），拟合被带偏
        const bool fresh = (pm.frames != g_pwrFrames);
        g_pwrFrames = pm.frames;

        // 掉线判据不能只看电压：掉线后电压会永远停在最后一帧的值，
        // 必须看数据年龄（年龄与数值在同一个 seqlock 快照里，不会错帧）
        const uint32_t ageMs = k_uptime_get_32() - pm.stamp_ms;
        const bool linkOk = (pm.bus_volt > kPwrMeterMinBusVolt)
                         && (ageMs <= kPwrMeterTimeoutMs);

        // 恢复滞回：连续 kPwrMeterResumeFrames 帧正常才算可用。否则 meterOk
        // 每拍抖动会让 EnableRls() 反复复位协方差，K1/K2 永远收敛不了
        g_pwrOkStreak = (linkOk && g_pwrOkStreak < kPwrMeterResumeFrames)
                      ? static_cast<uint8_t>(g_pwrOkStreak + 1) : 0;
        meterOk = (g_pwrOkStreak >= kPwrMeterResumeFrames);

        if (meterOk) {
            ChassisPwrCtrl.SetMeasuredPower(pm.power, fresh);
        }
    }
#endif

    // 掉线冻结辨识，沿用最后一次 K1/K2 开环预测
    ChassisPwrCtrl.EnableRls(kRlsEnable && meterOk);

    ChassisPwrCtrl.Predict();
    ChassisPwrCtrl.Allocate(kPowerBudget);

    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        g_wheelCurrent[i] = ChassisPwrCtrl.GetLimitedCurrent(i);
    }

    // ---- 限幅结果折回速度环积分（反算抗饱和）----
    // 限幅挂在速度环外面，PID 不知道自己被削了多少，长时间限流会把积分顶满
    // （iClamp = iOutMax/ki 相当于 20 A），预算一松开就窜一下。
    // cut/ki 正是"让输出降低 cut 所需的积分变化量"，所以这里直接扣掉。
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        const float cut = ChassisPwrCtrl.GetTargetCurrent(i) - g_wheelCurrent[i];
        if (cut == 0.0f || kPwrAntiWindupGain <= 0.0f) {
            continue;
        }

        const float ki = chassis_motor_omega_pid[i].GetKi();
        if (ki == 0.0f) {
            continue;
        }

        chassis_motor_omega_pid[i].SetIntegralError(
            chassis_motor_omega_pid[i].GetIntegralError() - kPwrAntiWindupGain * cut / ki);
    }
}

/**
 * @brief 组帧 → 投入 CAN 发送队列（消费者：本线程的 FlushTx）
 *  此外，再发送数据到上板
 */
static void FramePublish()
{
    topic::to_can_tx::Message msg {};
    msg.tx_id = kChassisTxId;

    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        // 电调量程固定 ±20A ↔ ±16384，这里按 kCurrentMax 再限一次（越界会整数回绕）
        const float    amp = std::clamp(g_wheelCurrent[i], -kCurrentMax, kCurrentMax);
        const uint16_t raw = static_cast<uint16_t>(static_cast<int16_t>(amp * kCurrentScale));

        msg.data[i * 2 + 0] = static_cast<uint8_t>(raw >> 8);     // 高字节在前（C620 协议）
        msg.data[i * 2 + 1] = static_cast<uint8_t>(raw & 0xFF);
    }

    if (k_msgq_put(chassis_tx, &msg, K_NO_WAIT) != 0) {
        // FlushTx 来不及取帧（正常不会发生）：丢当前帧并计数，控制环不阻塞
        if ((++g_txDrop % kTxDropLogPeriod) == 1u) {
            LOG_WRN("can tx queue full: %u frames dropped", static_cast<unsigned>(g_txDrop));
        }
    }
}

/**
 * @brief 外发排在 user_can1_msgq 里的帧
 *
 * 本线程是 user-can1 总线所有者：总线对象、收帧分发入口、外发都在本线程内，
 * 不再外包给别的线程。队列仍对外开放 —— 其它线程把帧 put 进来即可，
 * 本线程每拍取走（每拍限量、Send 用 K_NO_WAIT，总线异常或队列堆积都不阻塞 1ms 控制环）。
 */
static void FlushTx()
{
    topic::to_can_tx::Message msg {};
    uint8_t sent = 0;

    while (sent < kTxPerTickMax && k_msgq_get(chassis_tx, &msg, K_NO_WAIT) == 0)
    {
        // 契约固定 8 字节载荷（不能用 sizeof(tx.data)：CAN FD 模式下是 64 字节）
        can_frame tx {};
        tx.id  = msg.tx_id;
        tx.dlc = 8;
        memcpy(tx.data, msg.data, sizeof(msg.data));

        if (!user_can1.Send(&tx)) {
            if ((++g_txFail % kTxDropLogPeriod) == 1u) {
                LOG_WRN("can1 send fail id=0x%03x (total %u)",
                        static_cast<unsigned>(msg.tx_id), static_cast<unsigned>(g_txFail));
            }
        }
        ++sent;
    }
}

/**
 * @brief 速度环 + 功率限制整定波形 → VOFA+（JustFloat，最多 16 通道）
 *
 * 通道 2/3 是"限幅前 / 限幅后"的一对 —— 只有两个都看，才知道功率限制削了多少；
 * 通道 5/6 分别是实测口径与指令口径的预测总功率（判决用的是 6），7 是功率计实测值。
 * 调试完记得把这个每拍发送关掉（12 通道 @1kHz ≈ 52 B/ms，占串口约 56%）。
 */
[[maybe_unused]] static void VofaPublish()
{
    static uint8_t count = 0;

    if(++count < 5)
    {
        return;
    }

    count = 0;

    const float frame[] = {
        static_cast<float>(k_uptime_get_32()),
        // g_wheelTarget[kVofaWheel],                      
        g_wheelOmegaMeas[kVofaWheel],               
        g_wheelCurrent[kVofaWheel],                      
        ChassisPwrCtrl.GetTargetCurrent(kVofaWheel),  
        g_wheelCurrentMeas[kVofaWheel],
        g_wheelCurrentMeas[kVofaWheel + 1], 
        g_wheelCurrentMeas[kVofaWheel + 2],
        g_wheelCurrentMeas[kVofaWheel + 3],        
        ChassisPwrCtrl.GetTotalPower(),                  
        ChassisPwrCtrl.GetTotalPowerCmd(),              
        ChassisPwrMeter.GetPower(),

        ChassisPwrMeter.GetBusVolt(),
        ChassisPwrMeter.GetCurrent(),
        ChassisPwrMeter.GetBusVolt() * ChassisPwrMeter.GetCurrent(),
        // ChassisPwrCtrl.GetK1(),                           
        // ChassisPwrCtrl.GetK2(),                         
        ChassisPwrCtrl.GetMembership(kVofaWheel),        
        ChassisPwrCtrl.GetPowerLimit(kVofaWheel),         
    };

    vofa::Send(frame, sizeof(frame) / sizeof(frame[0]));
}

/**
 * @brief 用指定的静态损耗 k3 配置功率控制器
 *
 * k3 是模型里唯一能直接实测的参数（见 CalibrateK3()），所以把它做成入参，
 * 上电自校准前后各调用一次。
 */
static void ApplyPowerConfig(float k3)
{
    alg::power_ctrl::PowerCtrl<kMotorCount>::Config cfg {};
    cfg.torqueK        = kTorqueK;      // 与电机模块同一 Kt
    cfg.k1Init         = kPwrK1Init;    // = R / Kt²，见文件上方说明
    cfg.k2Init         = kPwrK2Init;
    cfg.k3             = k3;
    cfg.etaInit        = kPwrEta;       // = 1，不要再乘效率（见 kPwrEta 的说明）
    cfg.errUpper       = kPwrErrUpper;  // 与四轮 |err| 之和比较，不是单台误差
    cfg.errLower       = kPwrErrLower;
    cfg.rlsLambda      = kPwrRlsLambda; // 只在功率计新帧（500Hz）上迭代
    cfg.rlsEnable      = kRlsEnable;
    cfg.tauOmegaEnable = true;          // 保留 τ·ω 项
    cfg.minBudgetShare = kPwrMinBudgetShare;
    cfg.regenMaxW      = kPwrRegenMaxW; // 回馈功率上限（保护母线电压）
    cfg.k1Min          = kPwrK1Min;     // RLS 只允许在实测/物理值附近微调
    cfg.k1Max          = kPwrK1Max;
    cfg.k2Min          = kPwrK2Min;
    cfg.k2Max          = kPwrK2Max;
    ChassisPwrCtrl.Init(cfg);
}

/**
 * @brief 上电自校准：实测底盘静态损耗 k3
 *
 */
static void CalibrateK3()
{
#if CONFIG_USE_POWERMETER
    for (uint8_t attempt = 0; attempt < kK3CalibTries; ++attempt)
    {
        // 1) 等第一帧（以进入时的帧号为基准，上电时为 0）
        const uint32_t baseFrames = ChassisPwrMeter.ReadAll().frames;
        const uint32_t t0 = k_uptime_get_32();

        while (ChassisPwrMeter.ReadAll().frames == baseFrames)
        {
            if ((k_uptime_get_32() - t0) > kK3CalibWaitMs) {
                LOG_WRN("k3 calib: no frame in %u ms (try %u/%u)",
                        static_cast<unsigned>(kK3CalibWaitMs),
                        static_cast<unsigned>(attempt + 1u),
                        static_cast<unsigned>(kK3CalibTries));
                break;
            }
            k_msleep(1);
        }
        if (ChassisPwrMeter.ReadAll().frames == baseFrames) {
            k_msleep(1000);                       // 这一轮没等到帧，下次再来
            continue;
        }

        // 2) 第一帧到达之后再开始平均；只认**新帧**，同一帧的重复值不重复计入
        double   sum        = 0.0;
        uint32_t n          = 0;
        uint32_t lastFrames = baseFrames;
        const uint32_t tStart = k_uptime_get_32();

        while ((k_uptime_get_32() - tStart) < kK3CalibMs)
        {
            const auto pm = ChassisPwrMeter.ReadAll();
            if (pm.frames != lastFrames) {
                lastFrames = pm.frames;
                sum += static_cast<double>(pm.power);
                ++n;
            }
            k_msleep(1);
        }

        if (n == 0u) {
            LOG_WRN("k3 calib: no new frame while averaging (try %u/%u)",
                    static_cast<unsigned>(attempt + 1u),
                    static_cast<unsigned>(kK3CalibTries));
            k_msleep(1000);
            continue;
        }

        const float avgTotal = static_cast<float>(sum / static_cast<double>(n));

        // 3) 合理性检查：把"值为 0 的帧"和明显异常的值挡掉
        if (avgTotal < kK3CalibMinW || avgTotal > kK3CalibMaxW) {
            LOG_WRN("k3 calib: avg %u mW out of [%u, %u] mW (try %u/%u)",
                    static_cast<unsigned>(avgTotal * 1000.0f),
                    static_cast<unsigned>(kK3CalibMinW * 1000.0f),
                    static_cast<unsigned>(kK3CalibMaxW * 1000.0f),
                    static_cast<unsigned>(attempt + 1u),
                    static_cast<unsigned>(kK3CalibTries));
            k_msleep(1000);                       // 给功率计一点时间出有效值
            continue;
        }

        const float k3 = avgTotal / static_cast<float>(kMotorCount);
        LOG_INF("k3 calib: %u frames, avg %u mW total -> k3 %u mW/台",
                static_cast<unsigned>(n),
                static_cast<unsigned>(avgTotal * 1000.0f),
                static_cast<unsigned>(k3 * 1000.0f));
        ApplyPowerConfig(k3);
        return;
    }

    LOG_WRN("k3 calib failed, keep %u mW/台", static_cast<unsigned>(kPwrK3 * 1000.0f));
#else
    LOG_INF("k3 calib: no powermeter, keep %u mW/台",
            static_cast<unsigned>(kPwrK3 * 1000.0f));
#endif
}

/**
 * @brief 底盘控制主循环（周期 kPeriodMs）
 *
 */
static void Task(void*, void*, void*)
{
    // 上电自校准：此刻底盘还没动过，功率计读到的就是纯静态损耗 k3
    // （阻塞 ~0.5s；等功率计第一帧到达后才开始平均，见 CalibrateK3()）
    CalibrateK3();

    const int64_t period = k_ms_to_ticks_ceil64(kPeriodMs);
    int64_t deadline = k_uptime_ticks() + period;   // 下一个周期的绝对截止 tick

    for (;;)
    {
        ReadCommand();
        ReadGimbalYaw();
        UpdateTarget();

        ControlCalculate();
        PowerAllocate();
        FramePublish();
        FlushTx();
        // VofaPublish();

        // 睡到绝对截止时刻；本拍已经过点（循环体 > period）就直接重排基准，不往后欠债
        if (k_uptime_ticks() < deadline) {
            k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
        } else {
            deadline = k_uptime_ticks();
        }
        deadline += period;
    }
}

bool thread_init()
{
    // ---- user-can1：本线程是总线所有者 ----
    // Init（注册过滤器 + can_start）与收帧分发入口全局只能设一次，
    // 所以不再由 thread/can 代管：别的线程/组件不许再对本总线 Init()/SetRxCallback()。
    {
        const device *dev = DEVICE_DT_GET(DT_ALIAS(user_can1));
        if (!device_is_ready(dev)) {
            LOG_ERR("user_can1 not ready");
            return false;
        }

        const can_filter filter { .id = 0, .mask = 0, .flags = 0 };   // 全收，按 ID 分发
        if (!user_can1.Init(dev, filter)) {
            // 总线起不来 = 电机收不到反馈、控制帧也发不出去，按初始化失败处理
            LOG_ERR("user_can1 init fail");
            return false;
        }
        user_can1.SetRxCallback(user_can1_rx_callback);
        LOG_INF("user_can1 ready (%s)", dev->name);
    }

    const float dt = static_cast<float>(kPeriodMs) / 1000.0f;        // 与控制周期一致

    // 速度环：轮速 → 目标电流（输出限幅在电调量程内）；增益初值见文件上方 kSpeed*
    alg::pid::Pid::Config speed_cfg {};
    speed_cfg.kp      = kSpeedKp;
    speed_cfg.ki      = kSpeedKi;
    speed_cfg.kd      = 0.0f;
    speed_cfg.iOutMax = kCurrentMax;
    speed_cfg.outMax  = kCurrentMax;
    speed_cfg.dt      = dt;
    speed_cfg.dFirst  = alg::pid::DFirst::Enable;                   // 后续加 D 项时作用于测量值
    speed_cfg.iSeparateThresh = 3.0f;

    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        motor::dji::DjiC620::Config motor_cfg {};
        motor_cfg.rx_id         = kMotorRxId[i];
        motor_cfg.gearbox_ratio = kGearboxRatio;
        motor_cfg.torque_k      = kTorqueK;
        // 模块内 velocity = ω_out · wheel_r · 0.5，所以这里传轮径 = 2 × 轮半径
        motor_cfg.wheel_r       = 2.0f * kWheelRadius;

        chassis_motor[i].Init(motor_cfg);
        chassis_motor_omega_pid[i].Init(speed_cfg);
    }

    // ---- 功率限制：模型参数 + （可选）功率计 ----
    // k3 先用配置值兜底，线程起来后 CalibrateK3() 会实测覆盖（见那个函数的说明）
    ApplyPowerConfig(kPwrK3);
#if CONFIG_USE_POWERMETER
    ChassisPwrMeter.Init(kChassisPwrMeterId);
#endif

    // 启动摘要只打整数，省得每次上电刷一屏浮点；实时波形走 VOFA+
    LOG_INF("chassis ready: %u wheels, rx 0x%03x-0x%03x, tx 0x%03x, %u ms, iMax %u A",
            static_cast<unsigned>(kMotorCount),
            static_cast<unsigned>(kMotorRxId[0]),
            static_cast<unsigned>(kMotorRxId[kMotorCount - 1]),
            static_cast<unsigned>(kChassisTxId),
            static_cast<unsigned>(kPeriodMs),
            static_cast<unsigned>(kCurrentMax));
    LOG_INF("power limit: budget %u W, RLS %s", static_cast<unsigned>(kPowerBudget),
            kRlsEnable ? "on" : "off");
    // K1 决定低速/堵转能不能被限住，上电必须能一眼确认它不是默认占位值。
    // 这里的 K3 只是兜底值，线程起来后 CalibrateK3() 会用实测值覆盖
    LOG_INF("power model: K1 %u (x1e-3), K2 %u (x1e-6), K3 fallback %u W, eta %u (x0.1)",
            static_cast<unsigned>(kPwrK1Init * 1e3f),
            static_cast<unsigned>(kPwrK2Init * 1e6f),
            static_cast<unsigned>(kPwrK3),
            static_cast<unsigned>(kPwrEta * 10.0f));
    return true;
}

bool thread_start()
{
    thread_.Start(Task, ThreadPrio::High);
    return true;
}

// CAN 收帧注册：每个反馈 ID 一个入口（分发器只给帧数据，不给 ID）
CAN_RX_HANDLER(CHASSIS_RX_CAN, 0x201, [](uint8_t *data) { chassis_motor[0].CanCpltRxCallback(data); }, motor1);
CAN_RX_HANDLER(CHASSIS_RX_CAN, 0x202, [](uint8_t *data) { chassis_motor[1].CanCpltRxCallback(data); }, motor2);
CAN_RX_HANDLER(CHASSIS_RX_CAN, 0x203, [](uint8_t *data) { chassis_motor[2].CanCpltRxCallback(data); }, motor3);
CAN_RX_HANDLER(CHASSIS_RX_CAN, 0x204, [](uint8_t *data) { chassis_motor[3].CanCpltRxCallback(data); }, motor4);

// 功率计（与电机同总线；未开启时整体编译掉）
#if CONFIG_USE_POWERMETER
CAN_RX_HANDLER(CHASSIS_RX_CAN, kChassisPwrMeterId,
               [](uint8_t *data) { ChassisPwrMeter.CanCpltRxCallback(data); }, pwr_meter);
#endif

REGISTER_INIT  (thread_init,  MidInit,   Mid, "chassis_init");
REGISTER_THREAD(thread_start, MidThread, Mid, "chassis_start");

} // namespace thread::chassis
