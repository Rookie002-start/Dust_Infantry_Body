/**
 * @file trd_gimbal.hpp
 * @author Rookie002-start
 * @brief 云台实例集合：DM 电机对象 / 双环 PID / CAN ID 表 / TX 队列
 * @version 0.2
 * @date 2026-09-16
 *
 * 只被 thread/gimbal/trd_gimbal.cpp 使用：这里只放"实例与常量"。
 * CAN_RX_HANDLER 注册放在 .cpp —— 放在头文件里一旦被多处包含，
 * 会在 .can_rx3 段里重复登记同一个 ID，而且成员函数指针不能做段初值。
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include "Irq_handlers.h"
#include "to_can_tx.hpp"
#include "dm.hpp"
#include "pid.hpp"
#include <cstdint>

/// 云台电机所在 bus（user-can3）；本线程是总线所有者（Init + 收帧分发）
#define GIMBAL_RX USER_RX_CAN3

namespace instance::gimbal
{
    /// 控制帧去向：CAN 发送 topic（本线程入队后自己外发，队列对其它生产者同样开放）
    constexpr auto *gimbal_tx = &user_can3_msgq;

    // DM 电机 CAN ID：控制帧用 can_id 发出，反馈帧按 master_id 接收
    constexpr uint16_t kYawCanId      = 0x01;
    constexpr uint16_t kYawMasterId   = 0x00;
    constexpr uint16_t kPitchCanId    = 0x02;
    constexpr uint16_t kPitchMasterId = 0x11;

    /// 单轴：电机 + 双环 PID（外环角度 → 内环角速度 → 力矩）
    struct Axis {
        DmMotor       motor    {};
        alg::pid::Pid position {};   // 外环：角度 → 角速度给定 (rad/s)
        alg::pid::Pid omega    {};   // 内环：角速度 → 力矩给定 (N·m)
    };

    inline Axis yaw_   {};           // yaw 轴（可无限旋转）
    inline Axis pitch_ {};           // pitch 轴（有软限位）
}
