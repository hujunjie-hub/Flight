/* MPC task: 50 Hz MPC outer loop.
 * The 1 kHz rate loop stays inside the navigation task; this task
 * only advances the prediction model at the fixed 20 ms cadence. */
#include <rtthread.h>
#include "task_config.h"
#include "runtime_internal.h"
#include "control_interface.h"

static ControlBundle worker_bundle;

static void task_mpc_step(void)
{
    int ready = 0;
    runtime_data_mark_heartbeat(RUNTIME_OWNER_MPC, runtime_now_us());
    if (runtime_data_lock(0))
    {
        ready = runtime_data.bundle_ready;
        if (ready)
        {
            worker_bundle = runtime_data.bundle;
        }
        runtime_data_unlock();
    }
    mpc_note_worker(runtime_now_us(), ready);
    if (ready)
    {
        ControllerInput input;
        Status status;
        runtime_data_controller_input(&worker_bundle, &input, runtime_now_us());
        status = control_outer_update(&input);
        runtime_stats.mpc_updates++;
        if (status == STATUS_OK) {
            runtime_stats.mpc_solved++;
            runtime_stats.mpc_covered_us += (uint64_t)TASK_MPC_PERIOD_MS * 1000u;
        } else {
            runtime_stats.mpc_rejected++;
        }
    }
}

static uint8_t task_mpc_stack[8192] __attribute__((aligned(8)));

static void task_mpc_entry(void *parameter)
{
    rt_tick_t last = rt_tick_get();
    mpc_note_worker(runtime_now_us(), 2);
    (void)parameter;
    while (runtime_running()) {
        task_mpc_step();
        rt_thread_delay_until(&last, rt_tick_from_millisecond(TASK_MPC_PERIOD_MS));
    }
}

const TaskDescriptor task_mpc = {
    "mpc",
    task_mpc_entry,
    task_mpc_stack,
    sizeof(task_mpc_stack),
    TASK_MPC_PRIORITY,
};
