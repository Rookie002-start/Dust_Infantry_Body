/**
 * @file trd_inter_bus.hpp
 * @author Rookie002-start
 * @brief 上下板通信总线所有者（不建线程）—— 供发送/接收线程共用
 * @version 0.1
 * @date 2026-09-24
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
