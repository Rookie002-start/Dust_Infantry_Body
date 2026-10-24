#include "health.hpp"
#include "supervisor.hpp"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(wdt_supervisor, LOG_LEVEL_INF);

namespace {

const struct device *const kWatchdog =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_watchdog));
int channel = -1;
atomic_t fault_mask = ATOMIC_INIT(0);

#if CONFIG_WDT_SUPERVISOR_PRE_RESET_CALLBACK
void PreReset(const struct device *, int)
{
    LOG_ERR("watchdog pre-reset fault=0x%08x",
            static_cast<uint32_t>(atomic_get(&fault_mask)));
}
#endif

} // namespace

namespace wdt_supervisor {

bool Init()
{
    wdg_health::Start();

    if (!device_is_ready(kWatchdog)) {
        LOG_ERR("watchdog device not ready");
        return false;
    }

    const struct wdt_timeout_cfg cfg = {
        .window = {
            .min = 0,
            .max = CONFIG_WDT_SUPERVISOR_TIMEOUT_MS,
        },
#if CONFIG_WDT_SUPERVISOR_PRE_RESET_CALLBACK
        .callback = PreReset,
#else
        .callback = nullptr,
#endif
        .flags = WDT_FLAG_RESET_SOC,
    };

    channel = wdt_install_timeout(kWatchdog, &cfg);
    if (channel < 0) {
        LOG_ERR("watchdog install timeout failed: %d", channel);
        return false;
    }

    uint8_t options = WDT_OPT_PAUSE_HALTED_BY_DBG;
#if CONFIG_WDT_SUPERVISOR_PAUSE_IN_SLEEP
    options |= WDT_OPT_PAUSE_IN_SLEEP;
#endif
    if (wdt_setup(kWatchdog, options) != 0) {
        LOG_ERR("watchdog setup failed");
        return false;
    }
    return true;
}

void Run()
{
    const int64_t warmup_end =
        k_uptime_get() + CONFIG_WDT_SUPERVISOR_HEALTH_WARMUP_MS;
    bool fault_latched = false;

    for (;;) {
        if (!fault_latched && k_uptime_get() >= warmup_end) {
            const uint32_t faults = wdg_health::FaultMask();
            if (faults != 0U) {
                fault_latched = true;
                atomic_set(&fault_mask, faults);
                LOG_ERR("watchdog health fault mask=0x%08x", faults);
            }
        }

        if (!fault_latched && wdt_feed(kWatchdog, channel) != 0) {
            LOG_ERR("watchdog feed failed");
        }
        k_msleep(CONFIG_WDT_SUPERVISOR_FEED_PERIOD_MS);
    }
}

} // namespace wdt_supervisor
