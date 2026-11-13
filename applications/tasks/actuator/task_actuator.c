/* Actuation task: 1 ms output guard.
 * Applies a demand only after M7 re-validates the Permit at the frame
 * boundary; BENCH_INHIBIT keeps physical output disabled. */
#include <rtthread.h>
#include "task_config.h"
#include "runtime_internal.h"
#include "output_guard/actuator_guard.h"

static void task_actuator_step(void)
{
    ActuatorDemand demand;
    ActuatorStatus status;
    ActuatorCapability capability;
    ActuatorCommitReceipt receipt = {0};
    ActuationPermit permit;
    GroundStopCertificate ground_stop;
    int have_demand = 0;
    int have_ground_stop = 0;
    const uint64_t now_us = runtime_now_us();

    runtime_data_mark_heartbeat(RUNTIME_OWNER_ACTUATOR, runtime_now_us());
    if (runtime_data_lock(0))
    {
        if (runtime_data.bundle_ready)
        {
            ground_stop = runtime_data.bundle.ground_stop_certificate;
            have_ground_stop =
                ground_stop.header.message_type == MSG_GROUND_STOP_CERTIFICATE &&
                ground_stop.certificate_expires_at_us > now_us;
        }
        if (runtime_data.have_demand)
        {
            demand = runtime_data.demand;
            permit = runtime_data.bundle.permit;
            runtime_data.have_demand = 0u;
            have_demand = 1;
        }
        runtime_data_unlock();
    }
    /* The M4 certificate is a separate, post-revocation transaction. It
     * owns the actuator output while active; do not let a queued M6 demand race
     * the bounded ramp. Repeated copies are idempotent in the M7 guard. */
    if (have_ground_stop)
    {
        (void)actuator_guard_ground_stop(&ground_stop, runtime_now_us());
        have_demand = 0;
    }
    if (have_demand)
    {
        (void)actuator_apply(&demand, &permit, &status);
    }

    /* The 1 ms owner wake drives the M7 frame gate; the guard enforces the
     * minimum 2.5 ms inter-frame interval. BENCH_INHIBIT never reaches a driver. */
    if (!runtime_bench_inhibit_active())
        (void)actuator_guard_frame_boundary(runtime_now_us());

    /* Poll completes an asynchronous DMA frame before publishing receipt evidence. */
    (void)actuator_poll(runtime_now_us(), &status);
    (void)actuator_get_capability(&capability);
    (void)actuator_get_commit_receipt(&receipt);
    if (runtime_data_lock(0))
    {
        runtime_data.actuator = status;
        runtime_data.actuator_cap = capability;
        runtime_data.receipt = receipt;
        runtime_data_unlock();
    }
}

static uint8_t task_actuator_stack[2048] __attribute__((aligned(8)));

static void task_actuator_entry(void *parameter)
{
    rt_tick_t last = rt_tick_get();
    (void)parameter;
    while (runtime_running()) {
        task_actuator_step();
        rt_thread_delay_until(&last, rt_tick_from_millisecond(TASK_ACTUATOR_PERIOD_MS));
    }
}

const TaskDescriptor task_actuator = {
    "actuator",
    task_actuator_entry,
    task_actuator_stack,
    sizeof(task_actuator_stack),
    TASK_ACTUATOR_PRIORITY,
};
