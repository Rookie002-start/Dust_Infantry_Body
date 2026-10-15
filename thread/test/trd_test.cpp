#pragma message "Compiling Thread/Test"

#include "thread.hpp"
#include "Init_entry.hpp"
#include <zephyr/kernel.h>
#include "zephyr/zbus/zbus.h"
#include "vofa.hpp"
#include "imu_to.hpp"

namespace thread::test {

static Thread<2048> thread_{};

static void Task(void*, void*, void*)
{
    const float rad_to_deg = 180.0f / 3.14159265f;
    for (;;)
    {
        static topic::imu_to::Message msg{};
        if (zbus_chan_read(&pub_imu_to, &msg, K_NO_WAIT) != 0)
        {
            return;
        }
        float tx[] = {
            msg.gyro[0] * rad_to_deg,
            msg.gyro[1] * rad_to_deg,
            msg.gyro[2] * rad_to_deg,
            msg.temperature,
            // msg.roll,
            // msg.pitch,
            // msg.yaw,
            // msg.yaw_total
        };
        // printk("test\r\n");
        vofa::Send(tx, sizeof(tx) / sizeof(tx[0]));
        k_msleep(100);
    }
}

bool thread_init()
{
    return true;
}

bool thread_start()
{
    thread_.Start(Task, ThreadPrio::Low);
    return true;
}

REGISTER_INIT  (thread_init,  PreInit,   Low, "test_init");
REGISTER_THREAD(thread_start, PreThread, Low, "test_start");

}
