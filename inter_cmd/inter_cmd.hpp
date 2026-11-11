/**
 * @file inter_cmd.hpp
 * @author Rookie002-start
 * @brief 上下板通信接口契约（共享：任意业务线程发数据都走这里）
 *
 * 传输：**CAN FD 整帧**（FDF + BRS，数据段 2Mbps，DLC = can_bytes_to_dlc(帧长)）
 *   · Dir::Up   上板 → 下板：一帧，CAN ID 0x100（帧长 kUpFrameLen）
 *   · Dir::Down 下板 → 上板：一帧，CAN ID 0x200（帧长 kDownFrameLen）
 *   整帧一次发完，接收端无需拼包；一帧内所有分片来自同一拍，天然一致。
 *   要求两端都是 FDCAN 板（STM32F4 的 bxCAN 不支持 FD，不能用这套传输）。
 *
 * 只有状态分片，没有命令帧：业务线程 PostFrame() 分片 → 发送线程收集 →
 * 按 kStateFrags[] 表聚合成一帧 → 周期整帧发出。
 * 入队携带的 tag 只是"分片类型"标签，不是上线用的 CAN ID。
 *
 * 新增分片 = 加一枚举 + 一个结构体 + 表里加一行；帧长/校验/打包/解析全自动跟随。
 *
 * @version 0.6
 * @date 2026-09-14
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "to_mcu_tx.hpp"

namespace inter_cmd
{
    /// 通信方向
    enum class Dir : uint8_t
    {
        Up   = 0,   // 上板 → 下板
        Down = 1,   // 下板 → 上板
    };

    /// 状态分片类型注册表：协议文档同步维护
    enum class FrameType : uint8_t
    {
        // 上板 → 下板
        StateComm      = 0x20,   // CommState（底盘/云台指令汇总）
        StateAutoAim   = 0x21,   // AutoAimState
        StateImu       = 0x22,   // ImuState

        // 下板 → 上板
        StateGimbal    = 0x30,   // GimbalState（反馈角 + pitch 编码器角速度）
        StateShooter   = 0x31,   // ShooterState（子弹速度）
        StateTrace     = 0x32,   // TraceState（延迟诊断，仅 LATENCY_TRACE）
    };

    /// 底盘自转三态（CommState::chassis_spin 的取值，两端必须一致）
    enum class SpinMode : uint8_t
    {
        Normal = 0,   // 不转：自转速度取 chassis_rot
        Spin   = 1,   // 小陀螺：满速自转
        Stop   = 2,   // 上板要求停转
    };

    /// 聚合状态帧的 CAN ID（唯一事实来源）
    constexpr uint16_t StateTxId(Dir d)
    {
        return (d == Dir::Up) ? 0x100u : 0x200u;
    }

    // ============ 载荷结构体（packed：固定线缆布局，两端定义需完全一致）============

    /// 状态分片（Up）：底盘/云台指令汇总 + 上板数据源标志 + 开关状态位（23B，偏移 0）
    struct __attribute__((packed)) CommState
    {
        float   yaw_angle;      // 云台偏航角【指令】(rad)
        float   pitch_angle;    // 云台俯仰角【指令】(rad)
        float   chassis_vx;     // 底盘前后
        float   chassis_vy;     // 底盘左右
        float   chassis_rot;    // 底盘自转
        uint8_t chassis_spin;   // 小陀螺（三态）
        /// 上板数据源标志，见 kCommFlag*。下板必须先检查 kCommFlagLinkOk 再用上面的指令：
        /// 只看 CAN 帧有没有到是不够的（上板数据源掉线时帧照样在发，只是内容是旧值/零值）。
        uint8_t flags;
        /// 开关状态位（上板透传遥控开关；GCC 小端下从 bit0 开始排，两端编译器一致即可）
        /// ⚠️ 同样是"指令"，下板要配合 CommLinkOk() 判活后再采信
        struct
        {
            uint8_t Supercap       : 1;   // 超级电容：1-开
            uint8_t AutoAim        : 1;   // 自瞄开关：1-开
            uint8_t Gimbal_SetZero : 1;   // 云台归零：1-归零
            uint8_t Booster_Status : 1;   // 发射机构状态：1-已启动（摩擦轮开且未失能）
            uint8_t Fast_Run       : 1;   // 快跑：1-快跑
            uint8_t Refresh_UI     : 1;   // 刷新 UI：1-刷新
            uint8_t Reserved       : 2;
        } Switch;
    };

    /// CommState.flags 位定义（上板写入，下板读取）
    constexpr uint8_t kCommFlagLinkOk = 1u << 0;   // 遥控链路有效：速度/云台指令可信
    constexpr uint8_t kCommFlagImuOk  = 1u << 1;   // IMU 数据新鲜：ImuState 可信
    constexpr uint8_t kCommFlagVisionOk   = 1u << 2;   // PC 自瞄链路有效：AutoAimState 可信
    constexpr uint8_t kCommFlagVisionFire = 1u << 3;   // PC 请求开火（需 VisionOk 同时成立）
    /// 延迟诊断序号放在 flags[7:4]；正常固件不用这四位。
    constexpr uint8_t kCommFlagTraceSeqMask  = 0xF0u;
    constexpr uint8_t kCommFlagTraceSeqShift = 4u;

    inline bool CommLinkOk(const CommState &c) { return (c.flags & kCommFlagLinkOk) != 0u; }
    inline bool CommImuOk(const CommState &c)  { return (c.flags & kCommFlagImuOk)  != 0u; }
    inline bool CommVisionOk(const CommState &c)   { return (c.flags & kCommFlagVisionOk)   != 0u; }
    inline bool CommVisionFire(const CommState &c) { return (c.flags & kCommFlagVisionFire) != 0u; }
    inline uint8_t CommTraceSeq(const CommState &c)
    {
        return static_cast<uint8_t>((c.flags & kCommFlagTraceSeqMask) >>
                                    kCommFlagTraceSeqShift);
    }
    inline uint8_t TraceSeqBits(uint8_t seq)
    {
        return static_cast<uint8_t>((seq << kCommFlagTraceSeqShift) &
                                    kCommFlagTraceSeqMask);
    }

    /// 状态分片（Up）：自瞄数据（24B，偏移 23）
    /// 采信前必须先看 CommState.flags 的 kCommFlagVisionOk（PC 掉线/未锁定即清零）。
    struct __attribute__((packed)) AutoAimState
    {
        uint8_t yaw_angle[4];
        uint8_t yaw_omega[4];
        uint8_t yaw_accl[4];      // 角加速度
        uint8_t pitch_angle[4];
        uint8_t pitch_omega[4];
        uint8_t pitch_accl[4];    // 角加速度
    };

    /// 状态分片（Up）：IMU 数据（16B，偏移 47）
    struct __attribute__((packed)) ImuState
    {
        uint8_t total_yaw_angle[4];
        uint8_t pitch_angle[4];
        uint8_t yaw_omega[4];
        uint8_t pitch_omega[4];
    };

    /// 状态分片（Down）：云台电机反馈角 + pitch 编码器角速度（12B，偏移 0）
    /// ⚠️ 与 Up 方向 CommState 的 yaw/pitch 语义相反：那边是指令角，这边是电机反馈角
    struct __attribute__((packed)) GimbalState
    {
        float yaw;         // 云台电机反馈 yaw (rad)
        float pitch;       // 云台电机反馈 pitch (rad)
        float pitch_omega; // pitch 编码器反馈角速度（关节侧，rad/s）
    };

    /// 状态分片（Down）：子弹速度（4B，偏移 12）
    struct __attribute__((packed)) ShooterState
    {
        float bullet_speed;   // 单位按协议约定
    };

    /// 状态分片（Down）：自瞄延迟诊断，仅 CONFIG_LATENCY_TRACE=y 时进入下行帧。
    /// yaw_us/pitch_us 是下板本地 "CAN RX 中断 -> 云台控制帧提交" 的时间。
    struct __attribute__((packed)) TraceState
    {
        uint8_t  seq;
        uint8_t  valid_mask;
        uint16_t reserved;
        uint32_t yaw_us;
        uint32_t pitch_us;
    };

    constexpr uint8_t kTraceValidYaw   = 1u << 0;
    constexpr uint8_t kTraceValidPitch = 1u << 1;

    // ============ 聚合帧布局（唯一事实来源）============

    // 各分片在各自方向聚合帧中的偏移（协议事实，命名便于对照协议文档）
    constexpr uint8_t kOffComm    = 0;    // CommState    23B → 0..22
    constexpr uint8_t kOffAutoAim = 23;   // AutoAimState 24B → 23..46
    constexpr uint8_t kOffImu     = 47;   // ImuState     16B → 47..62

    constexpr uint8_t kOffGimbal  = 0;    // GimbalState   12B → 0..11
    constexpr uint8_t kOffShooter = 12;   // ShooterState   4B → 12..15
    constexpr uint8_t kOffTrace   = 16;   // TraceState    12B → 16..27（诊断构建）

    // 偏移必须与结构体长度严格对齐（两端协议改错时在这里直接编译失败）
    static_assert(kOffComm + sizeof(CommState) == kOffAutoAim,
                  "CommState 长度与 kOffAutoAim 不一致（改了结构体就要同步改偏移）");
    static_assert(kOffAutoAim + sizeof(AutoAimState) == kOffImu,
                  "AutoAimState 长度与 kOffImu 不一致（改了结构体就要同步改偏移）");
    static_assert(kOffShooter + sizeof(ShooterState) == kOffTrace,
                  "ShooterState 长度与 kOffTrace 不一致（改了结构体就要同步改偏移）");

    /// 一个状态分片的布局描述
    struct StateFrag
    {
        Dir       dir;      // 属于哪个方向
        FrameType type;     // 分片类型
        uint8_t   offset;   // 在该方向聚合帧中的字节偏移
        uint8_t   len;      // 分片长度
    };

    /// 聚合帧布局表
    constexpr StateFrag kStateFrags[] = {
        // 上板 → 下板
        { Dir::Up,   FrameType::StateComm,    kOffComm,    sizeof(CommState)    },
        { Dir::Up,   FrameType::StateAutoAim, kOffAutoAim, sizeof(AutoAimState) },
        { Dir::Up,   FrameType::StateImu,     kOffImu,     sizeof(ImuState)     },
        // 下板 → 上板
        { Dir::Down, FrameType::StateGimbal,  kOffGimbal,  sizeof(GimbalState)  },
        { Dir::Down, FrameType::StateShooter, kOffShooter, sizeof(ShooterState) },
#if defined(CONFIG_LATENCY_TRACE)
        { Dir::Down, FrameType::StateTrace,   kOffTrace,   sizeof(TraceState)   },
#endif
    };

    constexpr uint8_t kStateFragCount = sizeof(kStateFrags) / sizeof(kStateFrags[0]);

    /// 某方向聚合帧总长 = 该方向所有分片尾端的最大值（自动推导，无需手改）
    constexpr uint8_t FrameLen(Dir d)
    {
        uint8_t end = 0;
        for (uint8_t i = 0; i < kStateFragCount; ++i) {
            if (kStateFrags[i].dir != d) {
                continue;
            }
            const uint8_t e =
                static_cast<uint8_t>(kStateFrags[i].offset + kStateFrags[i].len);
            if (e > end) {
                end = e;
            }
        }
        return end;
    }

    constexpr uint8_t kUpFrameLen   = FrameLen(Dir::Up);     // 63
    constexpr uint8_t kDownFrameLen = FrameLen(Dir::Down);   // 16 / 28（诊断）

    /// 编译期校验：分片不得越界、同方向不得重叠、类型不得重复
    constexpr bool StateFragsValid()
    {
        for (uint8_t i = 0; i < kStateFragCount; ++i) {
            const uint16_t end_i = kStateFrags[i].offset + kStateFrags[i].len;
            if (end_i > 64) {
                return false;                                   // 超出 CAN FD 上限
            }
            for (uint8_t j = i + 1; j < kStateFragCount; ++j) {
                if (kStateFrags[i].type == kStateFrags[j].type) {
                    return false;                               // 类型重复，方向/偏移会歧义
                }
                if (kStateFrags[j].dir != kStateFrags[i].dir) {
                    continue;                                   // 不同方向不参与重叠判断
                }
                const uint16_t beg_j = kStateFrags[j].offset;
                const uint16_t end_j = beg_j + kStateFrags[j].len;
                if (beg_j < end_i && kStateFrags[i].offset < end_j) {
                    return false;                               // 同方向分片重叠
                }
            }
        }
        return true;
    }

    static_assert(kUpFrameLen   <= 64, "Up state frame exceeds CAN FD max 64B");
    static_assert(kDownFrameLen <= 64, "Down state frame exceeds CAN FD max 64B");
    static_assert(StateFragsValid(),   "state fragment layout invalid: overlap / out of range / dup");

    /// 从某方向的聚合帧里取出指定分片（接收侧用）
    template <typename T>
    inline bool GetFrag(const uint8_t *frame, Dir dir, FrameType type, T &out)
    {
        for (uint8_t i = 0; i < kStateFragCount; ++i) {
            if (kStateFrags[i].dir  != dir ||
                kStateFrags[i].type != type ||
                kStateFrags[i].len  != sizeof(T)) {
                continue;
            }
            memcpy(&out, frame + kStateFrags[i].offset, sizeof(T));
            return true;
        }
        return false;
    }

    // ============ 发送接口（多线程可并发调用，底层 k_msgq 多生产者）============

    /**
     * @brief 发布一个状态分片（发送线程收集后聚合成帧）
     * @note  载荷结构体须为裸数据（无指针/虚表），两端定义一致；
     *        浮点字段两端字节序/格式一致（都是小端 IEEE754 即安全）。
     */
    template <typename T>
    inline void PostFrame(FrameType type, const T &payload)
    {
        static_assert(sizeof(T) <= 64, "payload exceeds CAN FD max 64B");
        topic::to_mcu_tx::Message msg{};
        msg.tag = static_cast<uint8_t>(type);      // 分片类型标签（非上线 ID）
        msg.len = sizeof(T);
        memcpy(msg.data, &payload, sizeof(T));
        k_msgq_put(&user_can2_msgq, &msg, K_NO_WAIT);
    }
}
