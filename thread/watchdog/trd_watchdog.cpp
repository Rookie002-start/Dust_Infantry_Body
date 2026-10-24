#include "Init_entry.hpp"
#include "supervisor.hpp"
#include "thread.hpp"

namespace thread::wdg {

static Thread<1024> thread_;

static void Task(void*, void*, void*)
{
    wdt_supervisor::Run();
}

static bool thread_init()
{
    return wdt_supervisor::Init();
}

static bool thread_start()
{
    thread_.Start(Task, ThreadPrio::VeryLow);
    return true;
}

REGISTER_INIT  (thread_init,  MidInit,   Mid, "watchdog_init");
REGISTER_THREAD(thread_start, MidThread, Mid, "watchdog_start");

} // namespace thread::wdg
