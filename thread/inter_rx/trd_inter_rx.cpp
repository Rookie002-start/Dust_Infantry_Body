/**
 * @file trd_inter_rx.cpp
 * @author Rookie002-start
 * @brief 上板→下板接收线程（经典 CAN 拆帧）
 *        - 8 个分片帧（0x100 ~ 0x107）在回调里按固定偏移拼进聚合帧缓冲
 *        - 线程按发送周期读一份一致快照 → 解析 + 超时判定 → 发布 zbus
 *
 *
 * @version 0.6
 * @date 2026-09-15
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Inter_Rx"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "from_head.hpp"
#include "inter_cmd.hpp"
#include "Irq_handlers.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(inter_rx, LOG_LEVEL_INF);

// 链接段边界（cmd/linker/dust_init.ld 提供）。必须在全局作用域声明：
// 放到命名空间里会被 C++ 修饰成 thread::inter_rx::__can_rx2_start，链接找不到。
extern const CanRxEntry __can_rx2_start[], __can_rx2_end[];

namespace thread::inter_rx
{
    static Thread<2048> thread_{};

    static constexpr inter_cmd::Dir kDir         = inter_cmd::Dir::Up;    // 本线程接收方向
    static constexpr uint8_t        kFrameLen    = inter_cmd::FrameLen(kDir);    // 61
    static constexpr uint32_t       kRxTimeoutMs = 30;   // 发送周期 1ms → 连续丢 30 轮才判离线
    static constexpr uint32_t       kPollMs      = 1;    // 与对端发送周期一致

    // ---- ISR → 线程：聚合帧缓冲（spinlock 保证按字节不撕裂）----
    static struct
    {
        uint8_t           data[kFrameLen];
        struct k_spinlock lock;
        atomic_t          rx_cnt;         // 每收到一整帧 +1
    } asm_;

    // ==================== 中断上下文入口（只搬运，不解析）====================

    /// 整帧入口：CAN FD 一帧就是全部（不存在跨分片混合），整体拷进缓冲
    static void StateFrameHandler(uint8_t *data)
    {
        k_spinlock_key_t key = k_spin_lock(&asm_.lock);
        memcpy(asm_.data, data, sizeof(asm_.data));
        k_spin_unlock(&asm_.lock, key);

        atomic_inc(&asm_.rx_cnt);
    }

    // 一个方向一帧（CAN FD 整帧）
    CAN_RX_HANDLER(USER_RX_CAN2, inter_cmd::StateTxId(kDir), StateFrameHandler, up_frame);

    // ==================== 线程侧 ====================

    /// 按契约布局表解析并发布（方向/偏移/长度都来自 inter_cmd.hpp 的同一张表）
    static float RdF32(const uint8_t *p) { float v{}; memcpy(&v, p, sizeof v); return v; }

    static void Publish(const uint8_t *raw, bool online, uint16_t age_ms)
    {
        topic::from_head::Message msg{};
        (void)inter_cmd::GetFrag(raw, inter_cmd::Dir::Up, inter_cmd::FrameType::StateComm,    msg.comm);
        (void)inter_cmd::GetFrag(raw, inter_cmd::Dir::Up, inter_cmd::FrameType::StateAutoAim, msg.autoaim);
        (void)inter_cmd::GetFrag(raw, inter_cmd::Dir::Up, inter_cmd::FrameType::StateImu,     msg.imu);
        msg.online = online;
        msg.age_ms = age_ms;

        // 关键：先看 flags，再看数据；打印要限流，1ms 循环里 printk 会拖慢任务
        static int64_t last_dbg = 0;
        const int64_t now = k_uptime_get();
        if (now - last_dbg >= 500) {
            last_dbg = now;
            // printk("flags=0x%02x online=%d age=%u yaw=%.4f pitch=%.4f yaw_w=%.4f pitch_w=%.4f\r\n",
            //     msg.comm.flags, static_cast<int>(online), static_cast<unsigned>(age_ms),
            //     static_cast<double>(RdF32(msg.imu.total_yaw_angle)),
            //     static_cast<double>(RdF32(msg.imu.pitch_angle)),
            //     static_cast<double>(RdF32(msg.imu.yaw_omega)),
            //     static_cast<double>(RdF32(msg.imu.pitch_omega)));
            // printk("vx:%f", msg.comm.chassis_vx);
            // printk("vy:%f", msg.comm.chassis_vy);
            // printk("yaw:%f", msg.comm.yaw_angle);
            // printk("pitch:%f", msg.comm.pitch_angle);
            // printk("spin:%d", msg.comm.chassis_spin);
            
        }
        (void)zbus_chan_pub(&pub_from_head, &msg, K_NO_WAIT);
    }

    /// 启动自检：本方向必须正好注册 1 个整帧入口
    static bool FrameRegValid()
    {
        const uint16_t id = inter_cmd::StateTxId(kDir);
        uint8_t cnt = 0;
        for (const CanRxEntry *e = __can_rx2_start; e < __can_rx2_end; ++e) {
            if (e->id == id) {
                ++cnt;
            }
        }
        if (cnt != 1) {
            LOG_ERR("frame handlers = %u, expected 1 (注册行与实际不符?)",
                    static_cast<unsigned>(cnt));
            return false;
        }
        return true;
    }

    static void Task(void*, void*, void*)
    {
        uint8_t  buf[kFrameLen] {};
        uint32_t last_cnt    = 0;
        uint32_t last_rx_ms  = 0;
        bool     ever_rx     = false;
        bool     online_prev = false;

        for (;;)
        {
            const int64_t  tick  = k_uptime_get();
            const uint32_t cnt   = static_cast<uint32_t>(atomic_get(&asm_.rx_cnt));
            const bool     fresh = (cnt != last_cnt);
            const uint32_t now   = k_uptime_get_32();

            if (fresh)
            {
                k_spinlock_key_t key = k_spin_lock(&asm_.lock);
                memcpy(buf, asm_.data, sizeof(buf));
                k_spin_unlock(&asm_.lock, key);

                last_cnt   = cnt;
                last_rx_ms = now;
                ever_rx    = true;
            }

            const uint32_t age    = ever_rx ? (now - last_rx_ms) : UINT32_MAX;
            const bool     online = ever_rx && (age <= kRxTimeoutMs);

            // 有新分片、或在线状态翻转时发布；离线时保留最后一帧数据
            if (fresh || online != online_prev)
            {
                const uint16_t age16 = (age > UINT16_MAX) ? UINT16_MAX
                                                          : static_cast<uint16_t>(age);
                Publish(buf, online, age16);

                if (online != online_prev) {
                    LOG_INF("upper board %s (age=%ums)", online ? "online" : "TIMEOUT",
                            static_cast<unsigned>(age16));
                }
                online_prev = online;
            }

            const int64_t remain = static_cast<int64_t>(kPollMs) - (k_uptime_get() - tick);
            if (remain > 0) {
                k_msleep(remain);
            }
        }
    }

    bool thread_init()
    {
        // 总线 Init / 收帧分发入口由发送侧（thread/mcu_inter）负责
        if (!FrameRegValid()) {
            return false;
        }
        LOG_INF("inter_rx ready (dir=up, FD frame %uB, timeout=%ums)",
                static_cast<unsigned>(kFrameLen), kRxTimeoutMs);
        return true;
    }

    bool thread_start()
    {
        thread_.Start(Task, ThreadPrio::High);
        return true;
    }

    REGISTER_INIT  (thread_init,  MidInit,    High, "inter_rx_init");
    REGISTER_THREAD(thread_start, MidThread,  High, "inter_rx_start");
}
