/* I/O polling task: 250 Hz telemetry/GNSS/sensor poll owner.
 * Only drains transport and semantic queues; no control decision is made. */
#include <rtthread.h>
#include "task_comm.h"
#include "task_config.h"
#include "runtime_internal.h"
#include "sample_acquisition/sensor_input.h"
#include "telemetry_link/telemetry_link.h"

static void task_communication_step(void)
{
    uint64_t now = runtime_now_us();
    runtime_data_mark_heartbeat(RUNTIME_OWNER_COMMUNICATION, now);
#ifndef FLIGHT_SIM_LINK
    /* Native acquisition. A HIL build takes every measurement from the sim
     * link instead, so the real GNSS/barometer/magnetometer streams cannot be
     * mixed into the same estimator inputs (spec 02 section 4.9). */
    sensor_gnss_poll(now, runtime_submit_measurement);
    sensor_baro_mag_poll(runtime_now_us(), runtime_submit_measurement);
#endif
    comm_rc_poll();
    comm_radio_receive();
    comm_gcs_publish(now);
    comm_radio_transmit();
}

static void link_status(void)
{
    sensor_gnss_print_diagnostics();
    comm_rc_print_diagnostics();
    telemetry_link_print_diagnostics();
}
MSH_CMD_EXPORT(link_status, Flight native GNSS and telemetry diagnostics);

static uint8_t task_communication_stack[4096] __attribute__((aligned(8)));

static void task_communication_entry(void *parameter)
{
    rt_tick_t last = rt_tick_get();
    (void)parameter;
    while (runtime_running()) {
        task_communication_step();
        rt_thread_delay_until(&last, rt_tick_from_millisecond(TASK_COMMUNICATION_PERIOD_MS));
    }
}

const TaskDescriptor task_communication = {
    "telemetry",
    task_communication_entry,
    task_communication_stack,
    sizeof(task_communication_stack),
    TASK_COMMUNICATION_PRIORITY,
};
