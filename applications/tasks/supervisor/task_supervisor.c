/* Supervisor task: heartbeat audit and IWDG service at 10 Hz.
 * Stops output and deliberately stops feeding the watchdog when an owner
 * misses its deadline; it never performs control work itself. */
#include <rtthread.h>
#include "task_config.h"
#include "runtime_internal.h"
#include "device_adapter/actuator_device.h"
#include "drv_wdt.h"

static void task_supervisor_step(void)
{
    if (!runtime_running())
    {
        return;
    }
    for (RuntimeOwner owner = RUNTIME_OWNER_SENSOR; owner < RUNTIME_OWNER_COUNT; ++owner)
    {
        const uint64_t last = runtime_data_get_heartbeat(owner);
        /* An owner can preempt the supervisor and publish a new heartbeat. */
        const uint64_t now = runtime_now_us();
        if (now < last || now - last > 500000u)
        {
            actuator_device_inhibit();
            runtime_stats.deadline_misses++;
            runtime_stats.last_error = STATUS_DEADLINE_MISSED;
            if (runtime_stats.deadline_misses == 1u)
            {
                runtime_data_record_event(SOURCE_PLATFORM, 0, 6, 3, now);
            }
            return;
        }
    }
    watchdog_keepalive();
}

static uint8_t task_supervisor_stack[2048] __attribute__((aligned(8)));

static void task_supervisor_entry(void *parameter)
{
    (void)parameter;
    while (runtime_running()) {
        task_supervisor_step();
        rt_thread_mdelay(TASK_SUPERVISOR_PERIOD_MS);
    }
}

const TaskDescriptor task_supervisor = {
    "supervisor",
    task_supervisor_entry,
    task_supervisor_stack,
    sizeof(task_supervisor_stack),
    TASK_SUPERVISOR_PRIORITY,
};
