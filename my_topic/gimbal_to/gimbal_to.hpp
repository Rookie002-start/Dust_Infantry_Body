#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

namespace topic::gimbal_to {

struct Message {
    uint32_t version      = 0;      // 每次发布递增：判断数据是否更新 / 是否超时
    uint32_t timestamp_ms = 0;      // 发布时刻：上位机/健康判断用

    // 云台相对底盘（机械角，已含零位与方向修正）
    float yaw_rad    = 0.0f;        // 相对角度 (rad)

    // 云台电机反馈角（模式相关：IMU 或编码器）—— 供发送线程打包发给上板
    float yaw_fb_rad   = 0.0f;      // 电机反馈 yaw (rad)
    float pitch_fb_rad = 0.0f;      // 电机反馈 pitch (rad)

    bool  online     = false;       // 云台电机在线/无故障（供底盘降级策略）
};

}

ZBUS_CHAN_DECLARE(pub_gimbal_to);
