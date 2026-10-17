/**
 * @file trd_chassis.hpp
 * @author Rookie002-start
 * @brief 底盘实例集合：电机对象 / 速度环 PID / CAN 数据槽位表
 * @version 1.1
 * @date 2026-09-15
 *
 * 只被 thread/chassis/trd_chassis.cpp 使用：这里只放"实例与常量"，
 * 不放控制逻辑、不放 CAN_RX_HANDLER 注册（注册放在 .cpp，避免头文件
 * 被多处包含时在 .can_rx1 段里重复登记同一个 ID）。
 *
 * 每个轮子一组速度环 PID：输入输出轴轮速，输出目标电流（A）。
 * 增益初值来自 .cpp 里的整定常量（kSpeedKp / kSpeedKi），台架整定后抄回即可。
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include "Irq_handlers.h"
#include "to_can_tx.hpp"
#include "dji_c6xx.hpp"
#include "pid.hpp"
#include <cstdint>

#if CONFIG_USE_POWERMETER
#include "powermeter.hpp"
#endif

/// 底盘电机反馈所在 bus（user-can1）；本线程是总线所有者（Init + 收帧分发 + 外发）
#define CHASSIS_RX_CAN USER_RX_CAN1

namespace instance::chassis
{
    /// 控制帧去向：CAN 发送 topic（本线程入队后自己外发，队列对其它生产者同样开放）
    constexpr auto *chassis_tx = &user_can1_msgq;

#if CONFIG_USE_POWERMETER
    constexpr uint16_t kChassisPwrMeterId = 0x02;   ///< 功率计 CAN ID
    inline PowerMeter  ChassisPwrMeter {};          ///< 实测功率（seqlock）
#endif

    constexpr uint8_t  kMotorCount = 4;                                        ///< 轮数量
    constexpr uint16_t kMotorRxId[kMotorCount] = {0x201, 0x202, 0x203, 0x204}; ///< C620 反馈 ID

    inline motor::dji::DjiC620 chassis_motor[kMotorCount] {};                  ///< C620 + M3508
    inline alg::pid::Pid    chassis_motor_omega_pid[kMotorCount] {};           ///< 速度环：轮速 ω → 目标电流 A
}
