/**
 * @file trd_can_tx.cpp
 * @author Rookie002-start
 * @brief user-can1 总线所有者 + 控制帧发送泵
 *        - 初始化 user-can1（Init + 收帧分发入口），电机反馈经该过滤器进 .can_rx1
 *        - 从 topic::to_can_tx 队列取帧发出（多生产者 / 单消费者，满时生产者丢帧）
 * @version 0.2
 * @date 2026-09-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma message "Compiling Thread/Can"

#include "to_can_tx.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include "can.hpp"
#include "Irq_handlers.h"
#include <zephyr/logging/log.h>
#include "zephyr/drivers/gpio.h"




LOG_MODULE_REGISTER(can_tx, LOG_LEVEL_INF);

namespace thread::can {

// k_msgq vs zbus：zbus 多个发布者共用一个 channel 会互相覆盖；
// k_msgq 内部拷贝数据，多 put 一 get 天然支持多发布者，且满时丢帧不阻塞。
static Thread<> thread_{};
static Can user_can1{};
static uint32_t tx_fail = 0;                 // 发送失败计数（总线 bus-off / 邮箱满时诊断用）

static constexpr uint32_t kTxFailLogPeriod = 500;

static void Task(void*, void*, void*)
{
    for (;;)
    {
        topic::to_can_tx::Message msg{};

        // 契约固定 8 字节载荷（用 msg 的长度，不能用 sizeof(tx.data)：
        // CONFIG_CAN_FD_MODE=y 时 can_frame::data 是 64 字节）
        // 经典帧：flags 全 0，C620 只认经典帧
        can_frame tx{};
        tx.id  = 0x02;
        tx.dlc = 8;
        uint8_t tx_data[] = {0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFE};
        memcpy(tx.data, tx_data, sizeof(tx_data));

        if (!user_can1.Send(&tx)) {
            // 总线异常时每帧都会失败，限频上报，避免刷屏拖慢发送线程
            if ((++tx_fail % kTxFailLogPeriod) == 1u) {
                LOG_WRN("can send fail id=0x%03x (total %u)",
                        static_cast<unsigned>(msg.tx_id), static_cast<unsigned>(tx_fail));
            }
        }
        k_msleep(200);
    }
}

void my_rxcallback(struct can_frame &frame, void *)
{
    printk("can3callback");
}

bool thread_init()
{
    {
        const device* dev = DEVICE_DT_GET(DT_ALIAS(user_can3));
        if (!device_is_ready(dev)) {
            LOG_ERR("user_can1 not ready");
            return false;
        }
        const can_filter filter { .id = 0, .mask = 0, .flags = 0 };
        if (!user_can1.Init(dev, filter)) {
            // 总线起不来 = 电机收不到反馈、控制帧也发不出去，按初始化失败处理
            LOG_ERR("user_can1 init fail");
            return false;
        }
        user_can1.SetRxCallback(user_can1_rx_callback);
        user_can1.SetRxCallback(my_rxcallback);


        LOG_INF("user_can1 ready");
    }
    return true;
}

bool thread_start()
{
    thread_.Start(Task, ThreadPrio::High);
    return true;
}

REGISTER_INIT(thread_init,  PreInit,    High, "can_init");
REGISTER_THREAD(thread_start, PreThread,  High, "can_start");

} // namespace thread::can
