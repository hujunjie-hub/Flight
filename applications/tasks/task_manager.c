/* Application task manager.
 *
 * Creates, starts and tears down the RT-Thread threads described by the
 * per-task descriptors. Each task owns its stack and periodic work. */
#include <rtthread.h>
#include "task_config.h"
#include "runtime_internal.h"

static const TaskDescriptor *const g_tasks[] = {
    &task_sensor,
    &task_mission,
    &task_mpc,
    &task_actuator,
#ifdef FLIGHT_SIM_LINK
    &task_sim,
#endif
    &task_communication,
    &task_supervisor,
#ifdef FLIGHT_LOGGER_ENABLE
    &task_logger,
#endif
#ifdef FLIGHT_NOR_LOG_ENABLE
    &task_flash,
#endif
};

#define TASK_COUNT ((unsigned)(sizeof(g_tasks) / sizeof(g_tasks[0])))

static struct rt_thread g_threads[TASK_COUNT];

static void stop_all(unsigned count)
{
    while (count) rt_thread_detach(&g_threads[--count]);
}

int runtime_start(void)
{
    unsigned i;
    int result = runtime_initialize();
    if (result != RT_EOK) return result;
    for (i = 0; i < TASK_COUNT; ++i) {
        const TaskDescriptor *task = g_tasks[i];
        if (rt_thread_init(&g_threads[i], task->name, task->entry, 0, task->stack,
                           task->stack_size, task->priority, 1) != RT_EOK) {
            stop_all(i);
            runtime_shutdown();
            return -RT_ERROR;
        }
    }
    /* Release the ready tasks together. A failed startup cannot run part of
     * the pipeline while the task manager is rolling back. */
    rt_enter_critical();
    for (i = 0; i < TASK_COUNT; ++i) {
        if (rt_thread_startup(&g_threads[i]) != RT_EOK) {
            stop_all(TASK_COUNT);
            rt_exit_critical();
            runtime_shutdown();
            return -RT_ERROR;
        }
    }
    rt_exit_critical();
    return RT_EOK;
}
