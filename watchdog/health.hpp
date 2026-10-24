#pragma once

#include <cstdint>

namespace wdg_health {

enum class Service : uint8_t {
    Imu,
    Remote,
    Chassis,
    Gimbal,
    McuInter,
    InterRx,
    Count,
};

void Start();
void Mark(Service service);
uint32_t FaultMask();

} // namespace wdg_health
