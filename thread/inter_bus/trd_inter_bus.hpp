/**
 * @file trd_inter_bus.hpp
 * @author Rookie002-start
 * @brief 上下板通信总线所有者（不建线程）—— 供发送/接收线程共用
 * @version 0.1
 * @date 2026-09-24
 *
 * 这条总线被两个线程共用：发送（trd_mcu_inter）与接收（trd_inter_rx）。
 * 而 Can::Init()（注册过滤器 + can_start）与 SetRxCallback() 只能做一次，
 * 所以把它抽成一个只做初始化的组件，而不是绑在发送线程上——
 * 否则"只开接收"时总线不会被启动、收帧分发入口也没设，接收会静默失效。
 *
 * ⚠️ 只有本组件能对该总线调用 Can::Init() / SetRxCallback()。
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include "can.hpp"

namespace thread::inter_bus
{
    /// 上下板通信总线（DTS 别名 inter-can）。
    /// 使用方只调 Send()/SetTxCallback()，不要再 Init()/SetRxCallback()。
    Can& Bus();
}
