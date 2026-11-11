/**
 * @file from_head.hpp
 * @author Rookie002-start
 * @brief 上板→下板接收数据契约（topic 层：只定义结构体与通道，不含业务逻辑）
 *
 * 状态通道（zbus pub_from_head）：level 语义，保留最新值，供多个消费者各自读取。
 * （下板不使用命令帧，故此处没有命令队列）
 *
 * @version 0.1
 * @date 2026-09-14
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include "inter_cmd.hpp"      // 复用共享协议结构体与布局常量

namespace topic::from_head
{
    /// 状态帧契约（zbus 通道：消费者用 zbus_chan_read 轮询最新值）
    struct Message
    {
        inter_cmd::CommState    comm{};    // 底盘速度指令 / 云台角度指令
        inter_cmd::AutoAimState autoaim{};
        inter_cmd::ImuState     imu{};

        /// 超时判定结果。false = 上板数据已失效（上电初值为 false）
        ///  消费者必须先检查该标志，不要直接使用陈旧字段
        bool     online{};
        /// 注意：online 只表示"CAN 帧还有在到"。上板数据源（遥控/IMU）掉线时帧照样每 2ms 到，
        /// 只是内容变成失效值 —— 所以还要看 comm.flags：
        ///   inter_cmd::CommLinkOk(comm) 遥控链路有效（速度/云台指令可信）
        ///   inter_cmd::CommImuOk(comm)  IMU 数据新鲜（imu 字段可信）
        /// 判失效的完整条件：!online || !CommLinkOk(comm)
        /// 该帧年龄（ms），供消费者使用比接收侧更细的超时阈值
        uint16_t age_ms{};

#if defined(CONFIG_LATENCY_TRACE)
        /// 该 CAN 帧进入 RX 回调时的下板高分辨率周期，仅延迟诊断使用。
        uint32_t trace_rx_cycles{};
#endif
    };
}

ZBUS_CHAN_DECLARE(pub_from_head);
