#include "health.hpp"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

namespace {

constexpr uint32_t kServiceCount =
    static_cast<uint32_t>(wdg_health::Service::Count);
constexpr uint32_t kMissedChecks[kServiceCount] = {
    10, 15, 10, 10, 10, 10,
};
atomic_t counters[kServiceCount];
uint32_t previous[kServiceCount];
uint32_t missed[kServiceCount];

constexpr uint32_t Required(wdg_health::Service service)
{
    return BIT(static_cast<uint32_t>(service));
}

constexpr uint32_t kRequiredMask =
#if defined(CONFIG_TRD_IMU)
    Required(wdg_health::Service::Imu) |
#endif
#if defined(CONFIG_TRD_REMOTE)
    Required(wdg_health::Service::Remote) |
#endif
#if defined(CONFIG_TRD_CHASSIS)
    Required(wdg_health::Service::Chassis) |
#endif
#if defined(CONFIG_TRD_GIMBAL)
    Required(wdg_health::Service::Gimbal) |
#endif
#if defined(CONFIG_TRD_MCU_INTER)
    Required(wdg_health::Service::McuInter) |
#endif
#if defined(CONFIG_TRD_INTER_RX)
    Required(wdg_health::Service::InterRx) |
#endif
    0U;

} // namespace

namespace wdg_health {

void Start()
{
    for (uint32_t i = 0; i < kServiceCount; ++i) {
        previous[i] = static_cast<uint32_t>(atomic_get(&counters[i]));
        missed[i] = 0;
    }
}

void Mark(Service service)
{
    atomic_inc(&counters[static_cast<uint32_t>(service)]);
}

uint32_t FaultMask()
{
    uint32_t faults = 0;

    for (uint32_t i = 0; i < kServiceCount; ++i) {
        const uint32_t current =
            static_cast<uint32_t>(atomic_get(&counters[i]));
        if (current == previous[i]) {
            ++missed[i];
        } else {
            missed[i] = 0;
            previous[i] = current;
        }
        if ((kRequiredMask & BIT(i)) != 0U &&
            missed[i] >= kMissedChecks[i]) {
            faults |= BIT(i);
        }
    }
    return faults;
}

} // namespace wdg_health
