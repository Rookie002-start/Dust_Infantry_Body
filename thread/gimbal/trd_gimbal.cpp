/**
 * @file trd_gimbal.cpp
 * @author Rookie002-start
 * @brief 云台控制线程 — 1ms 固定周期：上板指令 → 双环 PID → DM 控制帧 → user-can3
 * @version 0.2
 * @date 2026-09-16
 *
 * ## 坐标与角度约定
 *
 *   yaw   ：绕 z 轴，逆时针为正，可无限旋转（虚拟角连续累加，不折叠）
 *   pitch ：抬头为正，软限位 [kPitchMin, kPitchMax]
 *
 * ## 数据来源与所有权
 *
 * | 数据 | 通道 / 段 | 生产者 | 本线程的角色 |
 * |------|-----------|--------|--------------|
 * | 目标角 | zbus `pub_from_head` | thread/inter_rx | 轮询读（帧断 / 遥控失效 → 停机、力矩 0） |
 * | IMU 角反馈 | `from_head` 的 `ImuState` 段（仅 IMU 模式） | 上板（随指令帧下发） | 轮询读 + flags 判活 |
 * | 编码器角反馈 | `.can_rx3` 段 | 本线程（总线所有者） | 只注册 CAN_RX_HANDLER |
 * | 电机控制帧 | k_msgq `user_can3_msgq` | 本线程 | 入队后由本线程外发 |
 * | 云台相对底盘角 | zbus `pub_gimbal_to` | 本线程 | 发布（底盘做云台系→车体系变换） |
 * | 电机反馈 yaw | topic `gimbal_to` | 本线程 | 发送线程拉取后打包发给上板 |
 *
 * 总线所有权：user-can3 由本线程持有并初始化（Init + 收帧分发入口），
 * 与 thread/can（can1）、thread/mcu_inter（can2）的约定一致。
 *
 * ## 控制流（每轴一组双环）
 *
 *     目标角(虚拟角) ─→ ⊖ ─→ [位置环] ─→ ω_ref ─→ ⊖ ─→ [角速度环] ─→ τ ─→ DM MIT 帧
 *                                ↑                          ↑
 *                        实测角(IMU/编码器)            实测 ω(电机反馈)
 *
 *   DM 侧 kp/kd = 0，闭环全在板端；力矩由内环直接给出。
 *
 * ## 反馈源：按轴 + 按环独立选（`kYawPosFb` / `kYawRateFb` / `kPitchPosFb` / `kPitchRateFb`）
 *
 *   · Encoder：电机编码器 —— "云台相对底盘"的关节角 + 关节角速度，1ms 更新、无融合延迟；
 *   · Imu：上板 IMU（`ImuState.total_yaw_angle` / `yaw_omega` / `pitch_angle` / `pitch_omega`，
 *     随上板指令帧一起下来）—— 世界系绝对角 + 陀螺角速度，底盘自转时云台可稳住指向。
 *     取上板而不是本板 imu_to：IMU 装在上板/云台侧，本板这路 IMU 的安装方向对不上。
 *     上板 IMU 判活看 CommState.flags 的 kCommFlagImuOk，失效即不再驱动电机。
 *
 * 参考系约束（重要）：同一轴的外环（角度）和内环（角速度）必须同参考系，否则内环的
 * 给定和反馈之间差一个"底盘角速度"，环路会一直带着静差/抖动。跨参考系时（外环世界系、
 * 内环关节系）必须靠 kYawChassisRateFfGain 前馈把给定折算过去。
 *
 * 当前选型（实测后锁定）：
 *   yaw   → IMU/IMU（世界系；陀螺 z 实测干净且与电机速度反馈一致到 3%）
 *   pitch → Encoder/Encoder（上板 pitch 陀螺链路实测脏：逐毫秒跳变超出机构物理极限）
 *
 * `CONFIG_TRD_GIMBAL_FB_IMU` 只决定"IMU 那套代码要不要编进来"，不再决定每轴选哪个源。
 *
 * ## 对齐沿（安全）
 *
 *   每轴记录上一拍的使能状态、反馈有效性与掉线/停机状态；出现下面四种"沿"时，
 *   把虚拟角直接对齐到实测角（IMU 或编码器）并清 PID 积分，避免目标与实际差
 *   一大截导致电机猛抽 / 撞限位：
 *     ① 电机「失能 → 使能」；
 *     ② 角度反馈「无效 → 有效」（IMU 掉线恢复等，期间云台可能是自由状态）。
 *     ③ 电机「掉线 → 恢复」（反馈帧超时期间不驱动，云台可能被外力带走）；
 *     ④ 上板「停机 → 解除」（停机期间力矩 0，pitch 可能已经掉下来）。
 *   对齐必须把"指令累计量"一起写回（yaw 的 `g_yaw_cmd`）：`UpdateTarget()` 每拍都拿指令
 *   重算虚拟角，只写虚拟角的话对齐只活 1ms —— 停机解除后 yaw 冲回虚拟 0 就是漏了这步。
 *   pitch 的指令是绝对量（摇杆居中 = 0°，见 ReadCommand），没有可重锚的累计量：停机解除后
 *   回到摇杆指令角是设计如此（操作员要的就是那个绝对角），不是漏对齐。
 *
 * ## 使能 / 失能 / 故障 / 掉线策略
 *
 *   · 上电使能：DM 上电默认失能，thread_init 初始化完两轴就各排一帧 Enable 命令帧
 *     （后续由下面的策略接管，不依赖控制环"顺手"发第一帧）；
 *   · 失能（err == Disable）：不闭环，每 kEnableResendTicks = 200ms 补发一帧使能命令，
 *     电机重新上电/复位后自动回到使能态；其余时间发 0 力矩控制帧喂通讯看门狗
 *     （停发帧会被电机判成"通讯丢失"，反而回不到使能态）；
 *   · 故障（超压/欠压/过流/MOS 过温/线圈过温/通讯丢失/过载/编码器未校准）：
 *     不闭环、不重发使能、不代替电机清故障（不发 ClearErr），只在故障码变化的沿
 *     上报一次（带故障名）；期间照样发 0 力矩控制帧保持通讯，故障消失回落到失能态后
 *     再由上面的策略自动使能；
 *   · 掉线（kLostTimeoutMs 内一帧反馈都没有 = 电机断电/断线）：不驱动，按"失能"
 *     同样周期性补发使能命令（不发 0 力矩帧：对端收不到，只会刷出无 ACK 错误帧）；
 *     这期间 snap 是旧值，绝不拿它闭环；
 *   · 对齐沿：失能→使能、掉线→恢复、反馈无效→有效、停机→解除，四种沿都把
 *     虚拟角对齐到当前实测角并清积分，避免恢复瞬间以最大角速度冲回旧目标；
 *   · 角度反馈无效（IMU 模式）：力矩置 0 但保持发帧（不拿陈旧角度闭环）；
 *   · 上板停机位（CommState.chassis_spin == Stop）：力矩置 0（松力，不保持姿态），
 *     电机保持使能、帧照发；
 *   · 指令链路失效（两种都按"停机"处理，停机位 = 力矩 0 松力、电机保持使能、帧照发）：
 *     ① 上板还在发帧但 CommState.flags 里没有 LinkOk（上板明确告知它的遥控源掉线）；
 *     ② 上板 CAN 帧整个断掉（msg.online = false）。两种都不允许拿上一帧陈旧指令
 *     继续驱动云台；恢复后由"停机→解除"的对齐沿重新对位（对齐实测角 + 清积分），
 *     所以不会突然冲回停机前的目标。
 *
 *
 *
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Gimbal"

#include "from_head.hpp"
#include "gimbal_to.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "trd_gimbal.hpp"
#include "dbg.hpp"             // 公共调参：把两轴的内外环 PID 登记给 RTT 调参终端
#include "can.hpp"
#include "lpf.hpp"             // alg::filter::LowPassFilter（算法模块现成的一阶低通）
#include "vofa.hpp"

#include <algorithm>
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gimbal, LOG_LEVEL_INF);

namespace {

constexpr float kPi  = 3.14159265f;
constexpr float k2Pi = 2.0f * kPi;
constexpr float rad_to_deg = 180.0f / kPi;

/// 角度归一化到 [-π, π)
float NormalizeAngle(float a)
{
    a = fmodf(a, k2Pi);
    if (a >  kPi) a -= k2Pi;
    if (a < -kPi) a += k2Pi;
    return a;
}

/// 协议里的浮点字段都是 4 字节小端（与 inter_rx 的打包方式一致）：按字节读回 float。
/// 不能把 uint8_t[4] 直接当 float 解引用（对齐 + 严格别名都不成立）。
float RdFloat(const uint8_t *p)
{
    float v {};
    memcpy(&v, p, sizeof v);
    return v;
}

} // namespace

namespace thread::gimbal {

using namespace instance::gimbal;

static Thread<2048> thread_ {};
static Can can3 {};                          // user-can3 总线对象（本线程私有）

// ==================== 周期 ====================
static constexpr uint32_t kPeriodMs = 1;
static constexpr float    kDt      = static_cast<float>(kPeriodMs) / 1000.0f;

// ==================== 机械与限幅 ====================
static constexpr float kPitchMin    = -0.310f;                    // 关节侧 rad，抬头为正
static constexpr float kPitchMax    =  0.350f;
static constexpr float kJointRatio  = 1.0f;                      // 1:1 减速比（已确认）
static constexpr float kMotorVmax   = 30.0f;                      // 电机侧角速度上限 rad/s（DM VMAX）
// 电机/协议力矩标定：必须等于电机手册的 TMAX（反馈力矩解码与 MIT 帧编码都用它）
static constexpr float kMotorTmax   = 10.0f;
// 内环输出限幅（本线程实际发出的力矩上限，电机侧）——保护机构用，先给小值：
static constexpr float kTorqueLimit = 4.0f;
static_assert(kTorqueLimit <= kMotorTmax, "inner-loop torque limit exceeds motor TMAX");
// 位置环输出的是"关节侧"角速度给定，上限 = 电机侧上限 ÷ 减速比（否则会超出 VMAX）
static constexpr float kMaxOmegaRef = kMotorVmax / kJointRatio;   // = 3.0 rad/s（关节侧）

// ==================== PID 初值（yaw / pitch 各自一套，手动整定） ====================
//pitch encoder反馈:out_kp = 12.0f out_ki = 0.0f in_kp = 2.0f in_ki = 2.5f

static constexpr float kYawPosKp     = 8.0f;
static constexpr float kYawPosKi     = 0.0f;
static constexpr float kYawOmegaKp   = 1.5f;
static constexpr float kYawOmegaKi   = 2.5f;
static constexpr float kPitchPosKp   = 12.0f;
static constexpr float kPitchPosKi   = 0.0f;
static constexpr float kPitchOmegaKp = 2.0f;
static constexpr float kPitchOmegaKi = 2.5f;

// ==================== 角速度环反馈低通（alg::filter::LowPassFilter）====================
// 角速度环的反馈（IMU 陀螺 / 电机编码器）都有高频噪声，直接进环会被 kp 放大成抖动、
// 啸叫。这里在反馈进环前加一阶低通（截止频率 Hz，≤0 = 直通）。
// 两轴可以不同：编码器相对干净可以给高一些，上板 IMU 陀螺噪声大可以给低一些。
// ⚠️ 越低越平滑但相位滞后越大（100 Hz ≈ 1.6 ms），整定 kp 时要把这段滞后算进去。
static constexpr float kYawOmegaLpfHz   = 100.0f;
static constexpr float kPitchOmegaLpfHz = 100.0f;

// 方向极性（实测后锁定：单轴点动确认正方向）
static constexpr int8_t kYawMotorSign   = +1;
static constexpr int8_t kPitchMotorSign = +1;
#if CONFIG_TRD_GIMBAL_FB_IMU
static constexpr int8_t kYawImuSign     = -1;
static constexpr int8_t kPitchImuSign   = +1;
#endif

// ==================== 每轴反馈源（按轴、按环独立选）====================
// 一个轴的两个环各自选源，但必须满足上面的"参考系约束"：
//   · 内环取 IMU 陀螺（世界系角速度）时，外环也必须是世界系（IMU 绝对角）；
//   · 外环世界系 + 内环关节系（编码器）时，必须开底盘角速度前馈把给定折过来。
enum class FbSource : uint8_t { Encoder, Imu };

static constexpr FbSource kYawPosFb    = FbSource::Imu;      // yaw 外环：世界系绝对航向
static constexpr FbSource kYawRateFb   = FbSource::Imu;      // yaw 内环：世界系陀螺 z
static constexpr FbSource kPitchPosFb  = FbSource::Imu;      // pitch 外环：关节角
static constexpr FbSource kPitchRateFb = FbSource::Encoder;  // pitch 内环：关节角速度

/// 反馈源名字（启动日志用）
constexpr const char *FbName(FbSource s)
{
    return (s == FbSource::Imu) ? "IMU" : "ENC";
}

#if !CONFIG_TRD_GIMBAL_FB_IMU
static_assert(kYawPosFb == FbSource::Encoder && kYawRateFb == FbSource::Encoder &&
              kPitchPosFb == FbSource::Encoder && kPitchRateFb == FbSource::Encoder,
              "选了 FbSource::Imu，但 CONFIG_TRD_GIMBAL_FB_IMU=n（IMU 那套代码没编进来）");
#endif

// ==================== 底盘 yaw 角速度前馈 ====================
static constexpr float kYawChassisRateFfGain =
    (kYawPosFb == FbSource::Imu && kYawRateFb == FbSource::Encoder) ? -1.0f : 0.0f;
// 前馈用的底盘角速度估计低通（估计量 = IMU 陀螺 − 编码器速度，后者有量化噪声）
static constexpr float kYawChassisRateLpfHz = 50.0f;

// ==================== pitch 重力前馈 ====================
static constexpr float kPitchGravCos =  1.277f;
static constexpr float kPitchGravSin = -0.122f;

/// pitch 重力前馈力矩（关节系 N·m，抬头为正）
static float PitchGravityFeedforward(float ang)
{
    return kPitchGravCos * cosf(ang) + kPitchGravSin * sinf(ang);
}

// ==================== 周期计数与超时 ====================
static constexpr uint32_t kEnableResendTicks = 200;   // 失能时每 200ms 重发一次使能帧
// 电机掉线判定：DM 电机每个控制帧回一帧反馈（本环 1ms 一帧），50ms 一帧都没有
// 就是断电/断线；比"两个检测窗口内没有新帧"更稳（不受检测节奏和调度抖动影响）
static constexpr uint32_t kLostTimeoutMs     = 50;
// IMU 模式：上板指令帧/ImuState 的本地超时（上板侧自己的 IMU 超时是 100ms，
// 用 kCommFlagImuOk 转达；这里只管"帧还在不在流里"）
static constexpr uint32_t kImuTimeoutMs      = 20;
static constexpr uint8_t  kTxPerTickMax      = 8;     // 每拍最多外发帧数
static constexpr uint32_t kDropLogPeriod     = 1000;  // 丢帧/失败上报周期

// ==================== 本线程私有状态 ====================
struct AxisState {
    bool     enabled_prev  = false;   // 上一拍电机是否使能（检测使能沿）
    bool     fb_valid_prev = false;   // 上一拍角度反馈是否有效（检测反馈恢复沿）
    bool     stop_prev     = false;   // 上一拍是否处于停机（检测停机解除沿）
    bool     lost_prev     = false;   // 上一拍是否判定为掉线（检测掉线/恢复沿）
    DmErrorStatus err_prev = DmErrorStatus::Disable;  // 上一拍电机状态（检测使能/失能/故障沿）
    uint32_t disable_ticks = 0;       // 失能期间的重发计数
    // 角速度环反馈一阶低通（thread_init 里按 k*OmegaLpfHz 配置；复位见 ProcessAxis 对齐沿）
    alg::filter::LowPassFilter omega_lpf {};
};

static AxisState g_yaw_state   {};
static AxisState g_pitch_state {};

#define YAWSENSITIVE 0.008f

static uint8_t g_auto_aim_cmd = 0;
static float g_auto_aim_yaw_angle = 0.0f;
static float g_auto_aim_yaw_omega = 0.0f;
static float g_auto_aim_yaw_accl  = 0.0f;
static float g_auto_aim_pitch_angle = 0.0f;
static float g_auto_aim_pitch_omega = 0.0f;
static float g_auto_aim_pitch_accl  = 0.0f;
static float g_yaw_cmd      = 0.0f;   // 上板指令角（yaw 已折算到离虚拟角最近的等价值）
static float g_pitch_cmd    = 0.0f;   // 上板指令角（已限幅）
static float g_yaw_target   = 0.0f;   // 虚拟角：位置环的实际目标
static float g_pitch_target = 0.0f;
static float yaw_out_torque = 0.0f;   // yaw 内环输出力矩（电机侧 N·m，限幅后）
static float pitch_out_torque = 0.0f; // pitch 内环输出力矩（电机侧 N·m，限幅后）
static bool gimbal_stop    = false;   // 云台停机指令：上板 Stop 位，力矩置 0（松力不保位）
/// 反馈帧到达时刻（CAN ISR 里写、控制环里读；32bit 对齐读写本身原子，差一拍无害）
static volatile uint32_t g_yaw_rx_ms   = 0;
static volatile uint32_t g_pitch_rx_ms = 0;
static uint32_t g_gimbal_version = 0; // pub_gimbal_to 版本号
static uint32_t g_drop = 0;           // 队列满丢帧计数
static uint32_t g_tx_fail = 0;        // CAN 发送失败计数

// 底盘 yaw 角速度估计（rad/s，物理系）：给 kYawChassisRateFfGain 用；两路反馈都新鲜才更新
static float g_chassis_yaw_rate = 0.0f;
static alg::filter::LowPassFilter g_chassis_yaw_lpf {};

/**
 * @brief 单轴一次控制周期的角度 + 角速度反馈（物理系：逆时针 / 抬头为正）
 *
 * ang / omega 都是物理系（逆时针 / 抬头为正），具体取哪个源由 k*PosFb / k*RateFb 决定：
 *   · Encoder 源：关节角、关节角速度（相对底盘），电机侧符号在这里折算；
 *   · Imu 源    ：世界系绝对角、陀螺角速度（同帧，k*ImuSign 折到物理系）。
 * 电机侧的极性换算统一放在 ProcessAxis 的输出处（k*MotorSign）。
 */
struct Feedback {
    float ang   = 0.0f;
    float omega = 0.0f;
    bool  valid = false;
};

// #define CONFIG_WAVE_TEST 1

// 测试波形选择：整定 PID 用阶跃(Step)，看跟随/频响用正弦(Sine)。改这一行切换。
enum class TestWave : uint8_t { Sine, Step };
static constexpr TestWave kTestWave = TestWave::Step;

// ==================== 扫频：每 5s 换一个频点 ====================
static constexpr float kSweepFreqs[] = {
    2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f,8.0f, 
    // 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f,
    // 15.0f, 16.0f, 17.0f, 18.0f, 19.0f, 20.0f
};
static constexpr uint32_t kSweepDwellMs = 5000;
static constexpr float    kSweepAmp     = 0.1f;    // rad，按轴调

static bool     g_sweep_on    = true;   // 触发开关
static float    g_sweep_phase = 0.0f;    // 连续相位
static uint32_t g_sweep_t0    = 0;       // 当前频点的开始时刻
static uint8_t  g_sweep_idx   = 0;

/**
 * @brief 扫频，每拍调用一次
 * @return 叠加在 base 上的扫频量（rad）
 * 
 */
static float SweepTarget(float base)
{
    constexpr uint8_t n = sizeof(kSweepFreqs) / sizeof(kSweepFreqs[0]);

    if (!g_sweep_on) {
        return base;
    }

    // ---- 到点换频点：t0 用累加而不是 = now，保证平均驻留正好 5s ----
    const uint32_t now = k_uptime_get_32();
    if (now - g_sweep_t0 >= kSweepDwellMs) {
        g_sweep_t0 += kSweepDwellMs;
        if (++g_sweep_idx >= n) {
            g_sweep_idx = 0;             // 扫完循环；想停就 g_sweep_on = false
        }
        LOG_INF("sweep -> %.2f Hz", (double)kSweepFreqs[g_sweep_idx]);
    }

    // ---- 相位累加：换频瞬间相位连续，不会跳相 ----
    g_sweep_phase += k2Pi * kSweepFreqs[g_sweep_idx] * kDt;
    if (g_sweep_phase >= k2Pi) {
        g_sweep_phase -= k2Pi;           // 别让它无限涨，float 久了会掉精度
    }

    return base + kSweepAmp * sinf(g_sweep_phase);
}

static constexpr float kSinFreqHz = 3.0f;    // 指定频率（Hz），按需修改
static constexpr float kSinAmp    = 0.2f;    // rad，按轴调

static bool  g_sin_on    = true;   // 触发开关
static float g_sin_phase = 0.0f;   // 连续相位

/**
 * @brief 固定频率正弦，每拍调用一次
 * @return 叠加在 base 上的正弦量（rad）
 *
 * 相位在 [0, 2π) 内累加并回绕，长时间运行不会因 float 精度掉相。
 * 每个控制周期只能调用一次：调用两次会让相位走两步、输出频率翻倍。
 */
static float SinTarget(float base)
{
    if (!g_sin_on) {
        return base;
    }

    g_sin_phase += k2Pi * kSinFreqHz * kDt;
    if (g_sin_phase >= k2Pi) {
        g_sin_phase -= k2Pi;
    }

    return base + kSinAmp * sinf(g_sin_phase);
}

// ==================== 阶跃给定（整定 PID 用） ====================
// 方波式阶跃：在 +kStepAmp 与 -kStepAmp 之间每 kStepHalfMs 跳一次。
// 看 VOFA 的"目标 vs 反馈"两条线就能读上升时间/超调/稳态误差。
static constexpr float    kStepAmp    = 0.20f;   // rad，阶跃高度（半幅）；pitch 别超软限位
static constexpr uint32_t kStepHalfMs = 1000;    // 每个电平保持多久（ms），半周期

static bool g_step_on = true;                     // 触发开关

/**
 * @brief 阶跃（方波）给定，每拍调用一次
 * @param base 叠加基准
 * @return base ± kStepAmp
 *
 * 用 k_uptime_get_32()（毫秒墙钟）计时，而不是数循环拍数：
 * 即使某一拍被拖后、或丢了几拍，阶跃沿的时刻也不会漂。
 * 与 SinTarget 同一约定 —— 每个控制周期只能调用一次。
 */
static float StepTarget(float base)
{
    if (!g_step_on) {
        return base;
    }

    // 第几个半周期；偶数半周期 +A，奇数半周期 -A，如此循环
    const uint32_t half = k_uptime_get_32() / kStepHalfMs;
    const float sign = (half & 1u) ? -1.0f : 1.0f;

    return base + sign * kStepAmp;
}


/**
 * @brief 读上板云台指令（角度）并更新指令角
 *
 * 状态通道 pub_from_head 由 thread/inter_rx 发布，定义为 ZBUS_OBSERVERS_EMPTY，
 * 因此用 zbus_chan_read 轮询（不占 subscriber）。
 *
 * 上板 CAN 帧断掉（msg.online = false）、或遥控链路失效（CommState.flags 没有 LinkOk）
 * 时都**停机**：不更新指令角，并置起 gimbal_stop → 力矩 0（松力不保位）、电机保持使能、
 * 帧照发。不允许拿上一帧陈旧指令继续驱动云台。指令恢复后由"停机→解除"的对齐沿
 * 重新对位（对齐实测角 + 清积分），不会突然冲回停机前的目标。
 */
static void ReadCommand()
{
    static topic::from_head::Message msg {};
    static bool online_prev = false;

    if (zbus_chan_read(&pub_from_head, &msg, K_NO_WAIT) != 0) {
        return;                                    // 读忙：沿用上一轮指令
    }

    const bool head_alive = msg.online;                       // 上板 CAN 帧还在不在
    const bool remote_ok  = inter_cmd::CommLinkOk(msg.comm);   // 上板自己的遥控数据是否有效
    const bool cmd_ok     = head_alive && remote_ok;

    if (!cmd_ok) {
        // 两种情况都停机（力矩 0，松力不保位）：
        //   · 上板还在发帧但遥控数据失效（flags 里没有 LinkOk）→ 操作手已经失控；
        //   · 上板 CAN 帧整个断（msg.online = false）→ 不知道上板会不会自己恢复，
        //     宁可让云台松力，也不允许拿上一帧陈旧指令继续驱动。
        if (online_prev) {
            LOG_WRN("gimbal command lost (head=%u link=%u flags=0x%02x) -> stop, torque 0",
                    head_alive ? 1u : 0u, remote_ok ? 1u : 0u,
                    static_cast<unsigned>(msg.comm.flags));
        }
        gimbal_stop = true;
        online_prev = false;
        return;
    }

    if (!online_prev) {
        LOG_INF("gimbal command online");
        online_prev = true;
    }

    // 停机位取协议契约里的三态枚举（与底盘同一读法），不是遥控 topic 的枚举：
    // 协议字段是 uint8_t，必须显式转换才能和 scoped enum 比较。
    const bool stop_now =
        (static_cast<inter_cmd::SpinMode>(msg.comm.chassis_spin) == inter_cmd::SpinMode::Stop);
    if (stop_now != gimbal_stop) {
        LOG_INF("gimbal stop %s (torque 0, still enabled)", stop_now ? "ON" : "OFF");
    }
    gimbal_stop = stop_now;

    // 自瞄生效：遥控自瞄开关 + 上板确认 PC 自瞄链路有效（VisionOk；PC 掉线/未锁定即清零）
    g_auto_aim_cmd         = (msg.comm.Switch.AutoAim != 0u) && inter_cmd::CommVisionOk(msg.comm);
    g_auto_aim_yaw_angle   = RdFloat(msg.autoaim.yaw_angle);
    g_auto_aim_yaw_omega   = RdFloat(msg.autoaim.yaw_omega);
    g_auto_aim_yaw_accl    = RdFloat(msg.autoaim.yaw_accl);
    g_auto_aim_pitch_angle = RdFloat(msg.autoaim.pitch_angle);
    g_auto_aim_pitch_omega = RdFloat(msg.autoaim.pitch_omega);
    g_auto_aim_pitch_accl  = RdFloat(msg.autoaim.pitch_accl);

    g_yaw_cmd   = g_yaw_cmd + msg.comm.yaw_angle * YAWSENSITIVE;
    g_pitch_cmd = msg.comm.pitch_angle;
    if(g_pitch_cmd < 0.0f)
    {
        g_pitch_cmd = g_pitch_cmd * -kPitchMin;
    }
    else
    {
        g_pitch_cmd = g_pitch_cmd * kPitchMax;
    }
}

/**
 * @brief 指令角 → 虚拟角（pitch 加软限位）
 *
 *    这里每拍都用指令重算虚拟角，所以 ProcessAxis 的对齐沿必须把"指令累计量"一起写回
 *    （yaw 的 g_yaw_cmd），只写虚拟角的话对齐只活 1ms。
 *    yaw 是指令增量累计（摇杆给角速度），pitch 是绝对指令（摇杆给角度）。
 */
static void UpdateTarget()
{
#ifndef CONFIG_WAVE_TEST
    if(g_auto_aim_cmd)
    {
        g_yaw_target   = g_yaw_cmd + g_auto_aim_yaw_angle;
        g_pitch_target = std::clamp(g_pitch_cmd + g_auto_aim_pitch_angle, kPitchMin, kPitchMax);
    }
    else
    {
        g_yaw_target   = g_yaw_cmd;
        g_pitch_target = std::clamp(g_pitch_cmd, kPitchMin, kPitchMax);
    }
#else
    const float test = (kTestWave == TestWave::Step) ? StepTarget(0.0f) : SinTarget(0.0f);
    g_yaw_target   = test;                       // yaw 要一起测时放开（无软限位）
    g_pitch_target = std::clamp(test, kPitchMin, kPitchMax);
#endif
}

// ==================== 角度反馈（每轴按 k*PosFb / k*RateFb 选源）====================

#if CONFIG_TRD_GIMBAL_FB_IMU
/**
 * @brief IMU 模式：取上板随指令帧一起下发的世界系角度
 *
 * 数据在 inter_cmd::ImuState（4 字节小端浮点）里，和 CommState 同一整帧从 0x100 下来，
 * 所以直接轮询 pub_from_head（状态通道无订阅者，读到的就是最新一帧）。
 *
 * 有效性要两层都成立，缺一不可：
 *   · 帧还在流：msg.online（inter_rx 判的 CAN 帧超时）且帧龄 age_ms 在 kImuTimeoutMs 内；
 *   · 上板 IMU 新鲜：CommState.flags 的 kCommFlagImuOk —— 上板 IMU 掉线时帧照样在发，
 *     只是内容是旧值，只看"帧到没到"会拿陈旧角度闭环。
 * 任一不成立即判无效 → 上层保持使能但力矩置 0（不拿陈旧角度闭环）。
 */
static Feedback ReadImuFeedback(bool yaw_axis)
{
    static topic::from_head::Message msg {};
    static uint32_t last_ms = 0;

    if (zbus_chan_read(&pub_from_head, &msg, K_NO_WAIT) == 0) {
        const bool ok = msg.online && inter_cmd::CommImuOk(msg.comm) &&
                        (msg.age_ms <= kImuTimeoutMs);
        last_ms = ok ? k_uptime_get_32() : 0u;
    }

    Feedback fb {};
    if (yaw_axis) {
        // 累计偏航可跨圈，不绕回；角速度取同一帧的陀螺 z 轴（上板已填到 yaw_omega）
        fb.ang   = kYawImuSign * RdFloat(msg.imu.total_yaw_angle);
        fb.omega = kYawImuSign * RdFloat(msg.imu.yaw_omega);
    } else {
        fb.ang   = kPitchImuSign * RdFloat(msg.imu.pitch_angle);
        fb.omega = kPitchImuSign * RdFloat(msg.imu.pitch_omega);
    }
    fb.valid = (last_ms != 0u) && ((k_uptime_get_32() - last_ms) <= kImuTimeoutMs);

    // 沿上报一次：上车台架时靠这条确认换源成功（帧在流 + 上板 IMU 新鲜）
    static bool valid_prev[2] = { false, false };
    const uint8_t idx = yaw_axis ? 0u : 1u;
    if (fb.valid != valid_prev[idx]) {
        valid_prev[idx] = fb.valid;
        LOG_INF("gimbal %s IMU feedback (upper board) %s",
                yaw_axis ? "yaw" : "pitch", fb.valid ? "online" : "STALE");
    }
    return fb;
}
#endif

/**
 * @brief 上板 IMU 反馈入口：IMU 那套代码没编进来时返回空对象
 *
 * 包一层是为了让 Task 里不同 #if 组合下的调用点长得一样（选源由 k*Fb 常量决定，
 * 不是这里）。
 */
static Feedback ReadImuFeedbackOrNone(bool yaw_axis)
{
#if CONFIG_TRD_GIMBAL_FB_IMU
    return ReadImuFeedback(yaw_axis);
#else
    (void)yaw_axis;
    return Feedback {};
#endif
}

/**
 * @brief 反馈入口：按每轴、每环的选源组装角度 + 角速度（物理系）
 *
 * @param snap     本拍电机快照（Encoder 源用；电机侧量 ÷ 减速比 = 关节角/关节角速度）
 * @param imu_fb   本拍上板 IMU 反馈（Imu 源用；IMU 代码没编进来时是空对象）
 * @param yaw_axis true = yaw
 *
 * 有效性：只要有一环取 Imu，就以 IMU 的新鲜度为准（不拿陈旧角度/角速度闭环）；
 * 全取 Encoder 时这里恒为有效，真正的判活交给上层的 snap.err / 掉线检测。
 */
static Feedback ReadFeedback(const DmMotor::Snapshot &snap, const Feedback &imu_fb, bool yaw_axis)
{
    const FbSource pos_src  = yaw_axis ? kYawPosFb   : kPitchPosFb;
    const FbSource rate_src = yaw_axis ? kYawRateFb  : kPitchRateFb;
    const int8_t   sign     = yaw_axis ? kYawMotorSign : kPitchMotorSign;

    Feedback fb {};
    fb.ang   = (pos_src == FbSource::Imu)
                   ? imu_fb.ang
                   : static_cast<float>(sign) * snap.radian / kJointRatio;
    fb.omega = (rate_src == FbSource::Imu)
                   ? imu_fb.omega
                   : static_cast<float>(sign) * snap.omega / kJointRatio;
    fb.valid = (pos_src == FbSource::Imu || rate_src == FbSource::Imu) ? imu_fb.valid : true;
    return fb;
}

/**
 * @brief 底盘 yaw 角速度估计（前馈用）
 *
 *   ψ_云台(世界) = ψ_底盘(世界) + θ_关节   ⇒   ω_底盘 = ω_云台(世界) − ω_关节
 *
 * ω_云台(世界) 取上板 IMU 陀螺 z（yaw_imu_fb.omega，已经过 kYawImuSign 折到物理系），
 * ω_关节 取 yaw 电机编码器角速度。任一路不新鲜 / 失能 / 掉线就清零并复位低通：
 * 前馈绝不能带陈旧值（推着云台跑的是力矩，不是显示）。
 */
static void UpdateChassisYawRate(const Feedback &yaw_imu_fb,
                                 const DmMotor::Snapshot &yaw_snap,
                                 bool yaw_lost)
{
    const bool ok = yaw_imu_fb.valid && !yaw_lost &&
                    (yaw_snap.err == DmErrorStatus::Enable);
    if (!ok) {
        g_chassis_yaw_rate = 0.0f;
        g_chassis_yaw_lpf.Reset(0.0f);
        return;
    }

    const float omega_joint =
        static_cast<float>(kYawMotorSign) * yaw_snap.omega / kJointRatio;
    g_chassis_yaw_rate = g_chassis_yaw_lpf.Update(yaw_imu_fb.omega - omega_joint);
}

// ==================== CAN 发送 ====================

static void QueueFrame(Axis &axis, bool cmd)
{
    topic::to_can_tx::Message msg {};
    msg.tx_id = axis.motor.GetTxId();

    if (cmd) {
        axis.motor.PackCmdFrame(msg.data, DmMotor::Cmd::Enable);
    } else {
        axis.motor.PackCtrlFrame(msg.data);
    }

    if (k_msgq_put(gimbal_tx, &msg, K_NO_WAIT) != 0) {
        if ((++g_drop % kDropLogPeriod) == 1u) {
            LOG_WRN("gimbal tx queue full: %u frames dropped",
                    static_cast<unsigned>(g_drop));
        }
    }
}

/// 排一帧使能命令（DM 上电默认失能：上电、复位、自保护失能后都靠它回到使能态）
static void RequestEnable(Axis &axis)
{
    QueueFrame(axis, true);
}

/**
 * @brief 外发本线程排在 user_can3_msgq 里的帧
 *
 * 本线程是 can3 总线所有者；队列同时对外开放，其它线程可以把帧投进来。
 */
static void FlushTx()
{
    topic::to_can_tx::Message msg {};
    uint8_t sent = 0;

    while (sent < kTxPerTickMax && k_msgq_get(gimbal_tx, &msg, K_NO_WAIT) == 0)
    {
        // 契约固定 8 字节载荷（不能用 sizeof(tx.data)：CAN FD 模式下是 64 字节）
        can_frame tx {};
        tx.id  = msg.tx_id;
        tx.dlc = 8;
        memcpy(tx.data, msg.data, sizeof(msg.data));

        if (!can3.Send(&tx)) {
            if ((++g_tx_fail % kDropLogPeriod) == 1u) {
                LOG_WRN("can3 send fail id=0x%02x (total %u)",
                        static_cast<unsigned>(msg.tx_id), static_cast<unsigned>(g_tx_fail));
            }
        }
        ++sent;
    }
}

// ==================== 电机状态检测 ====================

/// 掉线判定：kLostTimeoutMs 内一帧反馈都没有 → 电机断电/断线。
/// （DM 驱动里 PwrLossCheck() 是"两个检测窗口间有没有新帧"，需要对端按固定
///   周期调；这里直接用帧到达时刻，判断节奏和调度抖动都不会影响结果。）
static bool RxLost(volatile uint32_t &rx_ms)
{
    return (k_uptime_get_32() - rx_ms) > kLostTimeoutMs;
}

/// 故障码 → 可读名字（只在故障码变化的沿打印，1ms 环里不会刷屏）
static const char *ErrName(DmErrorStatus e)
{
    switch (e) {
    case DmErrorStatus::Disable:        return "disabled";
    case DmErrorStatus::Enable:         return "enabled";
    case DmErrorStatus::EncoderUncalib: return "encoder-uncalibrated";
    case DmErrorStatus::OverVoltage:    return "over-voltage";
    case DmErrorStatus::UnderVoltage:   return "under-voltage";
    case DmErrorStatus::OverCurrent:    return "over-current";
    case DmErrorStatus::MosOvertemp:    return "mos-overtemp";
    case DmErrorStatus::CoilOvertemp:   return "coil-overtemp";
    case DmErrorStatus::CommLost:       return "comm-lost";
    case DmErrorStatus::Overload:       return "overload";
    }
    return "unknown";
}

// ==================== 单轴控制 ====================

/**
 * @brief 单轴：使能管理 + 双环 PID
 *
 * @param axis         轴实例
 * @param st           轴的运行状态（使能沿）
 * @param sign         电机极性：物理正方向 → 电机正方向
 * @param target_angle 虚拟角（使能沿对齐时会被改写）
 * @param fb           本拍反馈：ang/omega 都是物理系（IMU 模式取上板 IMU 绝对角 + 陀螺，
 *                     编码器模式取电机编码器关节角 + 关节角速度），两者必须同源
 * @param stop         上板停机位：true = 力矩置 0（松力，不保持姿态）
 * @param lost         本拍电机掉线判定：true = 反馈帧超时（snap 是旧值，不许闭环）
 * @param snap         本拍电机快照（与反馈同帧，避免跨帧读数）
 * @param yaw_axis     true = yaw：对齐沿要把指令累计量 g_yaw_cmd 一起重锚（见下面的说明）
 * @param omega_ff     内环角速度给定前馈 (rad/s)：把外环所在参考系的给定折算到内环参考系，
 *                     外环世界系 + 内环关节系时 = −ω_底盘（见 kYawChassisRateFfGain）；
 *                     两环同参考系时传 0（默认）
 * @param tau_ff       内环力矩前馈 (N·m，关节系，抬头为正)：重力这类"已知负载力矩"直接
 *                     叠加在内环输出上。和 PID 输出一起限幅（不能绕过 kTorqueLimit）
 */
static void ProcessAxis(Axis &axis, AxisState &st, int8_t sign,
                        float &target_angle, const Feedback &fb,
                        bool stop, bool lost,
                        const DmMotor::Snapshot &snap,
                        bool yaw_axis, float &out_torque,
                        float omega_ff = 0.0f,
                        float tau_ff = 0.0f)
{
    const bool enabled = (snap.err == DmErrorStatus::Enable);

    // ---- 电机状态检测：反馈帧里的 err 半字节，只在变化的沿上报一次 ----
    if (snap.err != st.err_prev) {
        st.err_prev = snap.err;
        if (enabled) {
            LOG_INF("gimbal axis enabled");
        } else if (snap.err == DmErrorStatus::Disable) {
            LOG_WRN("gimbal axis disabled -> requesting enable");
        } else {
            LOG_ERR("gimbal axis fault: %s (0x%x)", ErrName(snap.err),
                    static_cast<unsigned>(snap.err));
        }
    }

    // ---- 掉线检测：反馈帧超时（电机断电/断线）----
    const bool lost_release = st.lost_prev && !lost;   // "掉线→恢复"这一沿
    if (lost != st.lost_prev) {
        st.lost_prev = lost;
        if (lost) {
            LOG_ERR("gimbal axis feedback LOST (%ums) -> hold, 0 torque",
                    static_cast<unsigned>(kLostTimeoutMs));
        } else {
            LOG_INF("gimbal axis feedback recovered");
        }
    }
    // ---- 对齐沿：失能→使能、掉线→恢复、反馈无效→有效、停机→解除 ----
    // 这几种情况下虚拟角都可能和实际差很远（停机/掉线期间电机不驱动，会被外力/重力带走），
    // 直接对齐 + 清积分，避免恢复瞬间以最大角速度冲回旧目标 / 撞限位
    const bool stop_release = st.stop_prev && !stop;
    const bool align_edge = (enabled && !st.enabled_prev) ||
                            (enabled && fb.valid &&
                             (!st.fb_valid_prev || stop_release || lost_release));
    if (align_edge) {
        if (fb.valid) {
            target_angle = fb.ang;
            // 对齐必须同时写回"指令累计量"，否则只生效 1ms：UpdateTarget() 每拍都拿 g_yaw_cmd
            // 重算虚拟角，而它只在操作员推摇杆时才变（停机期间手推云台不会改它）→ 下一拍虚拟角
            // 又回到旧目标，云台就冲回虚拟 0（这正是"stop 解除后 yaw 大幅度转到 0°"的原因）。
            // pitch 是指令绝对量（摇杆居中 = 0°），没有可重锚的累计量，保持原语义。
            if (yaw_axis) {
                g_yaw_cmd = fb.ang;
            }
            axis.position.SetIntegralError(0.0f);
            axis.omega.SetIntegralError(0.0f);
            st.omega_lpf.Reset(fb.omega);      // 低通状态也对齐到当前反馈，别从旧值慢慢爬
            const char *why = stop_release ? "stop released"
                                           : (lost_release ? "feedback recovered" : "ready");
            if (yaw_axis) {
                LOG_INF("gimbal yaw: %s, virtual angle re-anchored to %d mrad",
                        why, static_cast<int>(fb.ang * 1000.0f));
            } else {
                LOG_INF("gimbal pitch: %s (absolute command: follows stick next tick)", why);
            }
        } else {
            LOG_WRN("gimbal %s ready but feedback invalid -> hold", yaw_axis ? "yaw" : "pitch");
        }
    }
    st.enabled_prev = enabled;
    st.fb_valid_prev = fb.valid;
    st.stop_prev = stop;

    // 不闭环（输出 0 力矩），但这条 CAN 不能停：
    //   · 失能 / 故障：电机还活着，必须持续喂帧 —— DM 内部有通讯看门狗
    //     （几十 ms 收不到帧就报"通讯丢失"），停发帧会把"失能"变成"通讯丢失"；
    //   · 掉线：对端根本收不到，只按周期补发使能命令（电机重新上电即可自启），
    //     不发 0 力矩帧，避免无 ACK 的帧把总线刷成 error。
    // 补发使能只针对"失能"和"掉线"：故障由电机自己保持，这里不重发使能、
    // 也不代替它清故障（不发 ClearErr）；故障消失回落到失能态后自然会被重新使能。
    if (!enabled || lost)
    {
        const bool resend = ((st.disable_ticks++ % kEnableResendTicks) == 0u);
        if ((snap.err == DmErrorStatus::Disable || lost) && resend) {
            RequestEnable(axis);
        } else if (!lost) {
            axis.motor.SetTargetTorque(0.0f);   // 0 力矩控制帧：纯喂狗，不驱动
            QueueFrame(axis, false);
        }
        return;
    }

    st.disable_ticks = 0;               // 使能后重新计时，下次失能立即补发使能帧

    if (!fb.valid || stop)
    {
        // 反馈无效：给 0 力矩但保持发帧（不让电机带着陈旧角度闭环乱冲）
        axis.motor.SetTargetTorque(0.0f);
        QueueFrame(axis, false);
        return;
    }

#ifdef CONFIG_WAVE_TEST
    static float now_angle = 0;
    if(align_edge && fb.valid)
    {
        now_angle = fb.ang;
    }
    if(yaw_axis)    {target_angle += now_angle;}
#endif

    // 外环：角度 → 角速度给定（物理系）
    axis.position.SetTarget(target_angle);
    axis.position.SetNow(fb.ang);
    float omega_ref = std::clamp(axis.position.Calc(), -kMaxOmegaRef, kMaxOmegaRef);
    // 参考系换算前馈：外环世界系、内环关节系时才非零，把"世界系角速度给定"折成
    // "关节系角速度给定"；两环同参考系时恒为 0（见 kYawChassisRateFfGain）。
    if (omega_ff != 0.0f) {
        omega_ref = std::clamp(omega_ref + omega_ff, -kMaxOmegaRef, kMaxOmegaRef);
    }

    const float omega_fb = st.omega_lpf.Update(fb.omega);

    axis.omega.SetTarget(omega_ref);
    // axis.omega.SetTarget(target_angle);
    axis.omega.SetNow(omega_fb);
    // axis.omega.SetNow(fb.omega);

    const float torque_phys =
        std::clamp(axis.omega.Calc() + tau_ff, -kTorqueLimit, kTorqueLimit);
    const float torque = static_cast<float>(sign) * torque_phys;
    out_torque = torque;

    axis.motor.SetTargetTorque(torque);
    QueueFrame(axis, false);
}

// ==================== 对外发布 ====================

/**
 * @brief 发布云台角度：底盘要的"相对角" + 上板要的"实测角"
 *
 * @param fb            本拍反馈（模式相关：IMU 或编码器）
 * @param yaw_relative  云台相对底盘 yaw（始终来自编码器机械角，与反馈模式无关）
 * @param online        yaw 电机是否在使能态
 */
static void PublishAngles(const Feedback &yaw_fb, const Feedback &pitch_fb,
                          float yaw_relative, bool online)
{
    topic::gimbal_to::Message g {};
    g.version      = ++g_gimbal_version;
    g.timestamp_ms = k_uptime_get_32();
    g.yaw_rad      = NormalizeAngle(yaw_relative);
    g.yaw_fb_rad   = yaw_fb.ang;          // 上板要的云台反馈 yaw（模式相关：IMU 绝对角 / 编码器关节角）
    g.pitch_fb_rad = pitch_fb.ang;        // 上板要的云台反馈 pitch
    g.online       = online;
    (void)zbus_chan_pub(&pub_gimbal_to, &g, K_NO_WAIT);
}

static void VofaPublish(const DmMotor::Snapshot &pitch_snap, const DmMotor::Snapshot &yaw_snap, const Feedback &yaw_fb, const Feedback &pitch_fb)
{
    static uint32_t g_div = 0;

    if (++g_div != 10)
    {
        return;
    }
    g_div = 0;

    const float frame[] = {
        g_yaw_target * rad_to_deg,
        yaw_fb.ang * rad_to_deg,
        yaw_fb.omega * rad_to_deg,
        yaw_.position.GetOut() * rad_to_deg,
        yaw_snap.omega * rad_to_deg,
        yaw_snap.torque,

        // g_auto_aim_yaw_angle * rad_to_deg,
        // g_auto_aim_pitch_angle * rad_to_deg,

        // g_pitch_target * rad_to_deg,
        // pitch_fb.ang * rad_to_deg,
        pitch_fb.omega * rad_to_deg,
        // pitch_.position.GetOut() * rad_to_deg,
        // pitch_snap.radian * rad_to_deg,
        pitch_snap.omega * rad_to_deg,
        // pitch_out_torque
    };

    vofa::Send(frame, sizeof(frame) / sizeof(frame[0]));
}

/**
 * @brief 云台控制主循环（周期 kPeriodMs，绝对截止时刻计时）
 *
 * 周期必须靠"绝对截止时刻"维持：睡到 deadline，而不是 k_sleep(period - used)。
 * 后者里的 used = k_uptime_ticks() - tick_start 是按系统 tick（这里 0.5ms）量化的，
 * 循环体只要不足半拍就会读到 used = 0、于是多睡整整一拍 ——
 * 实测 1ms 目标跑成了 1.5ms（664Hz）。绝对截止时刻把 tick 量化误差和循环体
 * 耗时一起吸收掉，只要循环体 < period，平均周期就恒等于 period。
 * 循环体超时（> period）时不硬追，重排基准并计数，下条日志能看出来。
 */
static void Task(void*, void*, void*)
{
    const int64_t period = k_ms_to_ticks_ceil64(kPeriodMs);
    int64_t deadline = k_uptime_ticks() + period;   // 下一个周期的绝对截止 tick

    for (;;)
    {
        static uint32_t iters = 0, overruns = 0, body_max = 0, last_ms = 0;

        // 从循环最顶端开始计时：这样 LOG_INF 本身的耗时也会被 body_max 抓到
        //（日志后端是 UART/IMMEDIATE，写一行是同步阻塞的，见下面的说明）
        const int64_t body_t0 = k_uptime_ticks();

        ++iters;
        if (k_uptime_get_32() - last_ms >= 1000u) {
            last_ms = k_uptime_get_32();
            // LOG_INF("gimbal loop %u Hz, overrun %u, body_max %u ticks (%u us), vofa_drop %u",
            //         iters, overruns, body_max,
            //         static_cast<unsigned>((static_cast<uint64_t>(body_max) * 1000000u) /
            //                               CONFIG_SYS_CLOCK_TICKS_PER_SEC),
            //         vofa::DropCount());
            iters = 0;
            overruns = 0;
            body_max = 0;
        }

        ReadCommand();
        UpdateTarget();

        // 每轴一次快照：位置/速度/使能状态都取自同一帧
        const auto yaw_snap   = yaw_.motor.ReadAll();
        const auto pitch_snap = pitch_.motor.ReadAll();

        // 上板 IMU 反馈：yaw 内外环都用；pitch 走编码器（照样读出来交给 ReadFeedback 按源挑，
        // IMU 那套代码没编进来时是空对象）
        const Feedback yaw_imu_fb   = ReadImuFeedbackOrNone(true);
        const Feedback pitch_imu_fb = ReadImuFeedbackOrNone(false);

        // 按每轴反馈源组装：yaw = 世界系（IMU 绝对角 + 陀螺 z），pitch = 关节系（编码器）
        const Feedback yaw_fb   = ReadFeedback(yaw_snap,   yaw_imu_fb,   true);
        const Feedback pitch_fb = ReadFeedback(pitch_snap, pitch_imu_fb, false);

        // 掉线检测：DM 每收一个控制帧回一帧反馈，超时就是断电/断线（snap 已变成旧值）
        const bool yaw_lost   = RxLost(g_yaw_rx_ms);
        const bool pitch_lost = RxLost(g_pitch_rx_ms);

        // 底盘 yaw 角速度估计（前馈用）：ω_底盘 = ω_云台(世界系 IMU) − ω_关节(yaw 编码器)
        UpdateChassisYawRate(yaw_imu_fb, yaw_snap, yaw_lost);

        ProcessAxis(yaw_,   g_yaw_state,   kYawMotorSign,
                    g_yaw_target,   yaw_fb,   gimbal_stop, yaw_lost, yaw_snap,
                    true, yaw_out_torque, kYawChassisRateFfGain * g_chassis_yaw_rate);
        ProcessAxis(pitch_, g_pitch_state, kPitchMotorSign,
                    g_pitch_target, pitch_fb, gimbal_stop, pitch_lost, pitch_snap,
                    false, pitch_out_torque, 0.0f,
                    PitchGravityFeedforward(pitch_fb.ang));

        FlushTx();

        VofaPublish(pitch_snap, yaw_snap, yaw_fb, pitch_fb);

        const float yaw_relative =
            kYawMotorSign * yaw_snap.radian / kJointRatio;
        PublishAngles(yaw_fb, pitch_fb, yaw_relative,
                      (yaw_snap.err == DmErrorStatus::Enable) && !yaw_lost);

        // 循环体耗时（不含睡觉）：用来区分"体超时"和"被同等优先级线程拖后"
        const uint32_t body = static_cast<uint32_t>(k_uptime_ticks() - body_t0);
        if (body > body_max) {
            body_max = body;
        }

        // 睡到绝对截止时刻；本拍已经过点（循环体 > period）就直接重排基准，不往后欠债
        if (k_uptime_ticks() < deadline) {
            k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
        } else {
            ++overruns;
            deadline = k_uptime_ticks();
        }
        deadline += period;
    }
}

/**
 * @brief 单轴初始化：电机（DM MIT，kp/kd=0）+ 双环 PID
 */
static void InitAxis(Axis &axis, uint16_t can_id, uint16_t master_id,
                     const alg::pid::Pid::Config &pos_cfg,
                     const alg::pid::Pid::Config &omega_cfg)
{
    DmMotor::Config cfg {};
    cfg.ctrl_met      = ControlMethon::Mit;
    cfg.can_id        = can_id;
    cfg.master_id     = master_id;
    cfg.gearbox_ratio = kJointRatio;
    cfg.wheel_r       = 1.0f;               // 云台不做线速度换算
    cfg.kp            = 0.0f;               // 电机内置位置环关闭：闭环在板端
    cfg.kd            = 0.0f;
    cfg.PMAX          = 12.5f;
    cfg.VMAX          = kMotorVmax;
    cfg.TMAX          = kMotorTmax;         // 协议标定，不随内环限幅改动

    axis.motor.Init(cfg);
    axis.position.Init(pos_cfg);
    axis.omega.Init(omega_cfg);
}

bool thread_init()
{
    // ---- user-can3：本线程是总线所有者 ----
    const device *dev = DEVICE_DT_GET(DT_ALIAS(user_can3));
    if (!device_is_ready(dev)) {
        LOG_ERR("user_can3 not ready");
        return false;
    }

    const can_filter filter { .id = 0, .mask = 0, .flags = 0 };   // 全收，按 ID 分发
    if (!can3.Init(dev, filter)) {
        LOG_ERR("user_can3 init fail");
        return false;
    }
    can3.SetRxCallback(user_can3_rx_callback);

    // ---- 双环 PID（yaw / pitch 增益各自一套，取值见文件上方 k*PosKp / k*OmegaKp）----
    // 位置环：输出角速度给定，限幅 kMaxOmegaRef；角速度环：输出力矩给定，限幅 kTorqueLimit
    auto make_pos_cfg = [](float kp, float ki) {
        alg::pid::Pid::Config c {};
        c.kp      = kp;
        c.ki      = ki;
        c.kd      = 0.0f;
        c.iOutMax = kMaxOmegaRef;      // 积分限幅与输出限幅一致，防止饱和蓄积分
        c.outMax  = kMaxOmegaRef;
        c.dt      = kDt;
        c.dFirst  = alg::pid::DFirst::Enable;
        return c;
    };
    auto make_omega_cfg = [](float kp, float ki) {
        alg::pid::Pid::Config c {};
        c.kp      = kp;
        c.ki      = ki;
        c.kd      = 0.0f;
        c.iOutMax = kTorqueLimit / 2;
        c.outMax  = kTorqueLimit;      // 输出即力矩（电机侧），限幅保护机构
        c.dt      = kDt;
        c.dFirst  = alg::pid::DFirst::Enable;
        return c;
    };

    InitAxis(yaw_,   kYawCanId,   kYawMasterId,
             make_pos_cfg(kYawPosKp,     kYawPosKi),   make_omega_cfg(kYawOmegaKp,   kYawOmegaKi));
    InitAxis(pitch_, kPitchCanId, kPitchMasterId,
             make_pos_cfg(kPitchPosKp,   kPitchPosKi), make_omega_cfg(kPitchOmegaKp, kPitchOmegaKi));

    // ---- 角速度环反馈低通（每轴一个实例，采样周期 = 控制周期）----
    g_yaw_state.omega_lpf.Init(kYawOmegaLpfHz, kDt);
    g_pitch_state.omega_lpf.Init(kPitchOmegaLpfHz, kDt);
    // ---- 底盘 yaw 角速度估计低通（前馈用；估计量含编码器速度的量化噪声）----
    g_chassis_yaw_lpf.Init(kYawChassisRateLpfHz, kDt);

    // ---- 上电使能：DM 上电默认失能，先各排一帧 Enable 命令帧 ----
    // 排队即可（本线程是 can3 所有者，第一拍 FlushTx 就发出去）；后面电机没进使能态
    // 时控制环还会每 kEnableResendTicks 补发，所以这里发一次就够。
    RequestEnable(yaw_);
    RequestEnable(pitch_);

    // ---- 把两轴登记给公共调参终端（dbg 关着时是空操作，本线程不依赖 dbg）----
    // 用法：pid gimbal / pid gimbal yaw / pid gimbal yaw out <kp> <ki> / pid gimbal yaw in <kp> <ki>
    dbg::RegisterPid({ "gimbal", "yaw",   &yaw_.position,   &yaw_.omega   });
    dbg::RegisterPid({ "gimbal", "pitch", &pitch_.position, &pitch_.omega });

    LOG_INF("gimbal ready: yaw 0x%02x/0x%02x pitch 0x%02x/0x%02x, "
            "feedback yaw %s/%s pitch %s/%s (pos/rate), %u ms",
            static_cast<unsigned>(kYawCanId), static_cast<unsigned>(kYawMasterId),
            static_cast<unsigned>(kPitchCanId), static_cast<unsigned>(kPitchMasterId),
            FbName(kYawPosFb), FbName(kYawRateFb), FbName(kPitchPosFb), FbName(kPitchRateFb),
            static_cast<unsigned>(kPeriodMs));
    return true;
}

bool thread_start()
{
    thread_.Start(Task, ThreadPrio::High);
    return true;
}

// CAN 收帧注册：反馈帧按 master_id 分发，回调内再校验 payload 里的 can_id。
// 回调在 CAN 中断里跑：先记一帧到达时刻（掉线检测用），再交给驱动解析
CAN_RX_HANDLER(GIMBAL_RX, kYawMasterId, [](uint8_t *data) {
    g_yaw_rx_ms = k_uptime_get_32();
    yaw_.motor.CanCpltRxCallback(data);
}, yaw);
CAN_RX_HANDLER(GIMBAL_RX, kPitchMasterId, [](uint8_t *data) {
    g_pitch_rx_ms = k_uptime_get_32();
    pitch_.motor.CanCpltRxCallback(data);
}, pitch);

REGISTER_INIT  (thread_init,  MidInit,   Mid, "gimbal_init");
REGISTER_THREAD(thread_start, MidThread, Mid, "gimbal_start");

} // namespace thread::gimbal
