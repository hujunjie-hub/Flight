#include "device_adapter/actuator_device.h"
#include "dshot.h"
#include "dshot_packet.h"
#include "ports/port_api.h"

static rt_device_t dshot;
static uint32_t pending_frame_count;
static uint32_t pending_fault_count;
static uint8_t pending_apply;

Status actuator_device_start(void)
{
    rt_device_t device = rt_device_find(DSHOT_DEVICE_NAME);
    if (dshot || !device || rt_device_open(device, RT_DEVICE_OFLAG_WRONLY) != RT_EOK)
    {
        return STATUS_NOT_READY;
    }
    dshot = device;
    actuator_device_inhibit();
    return STATUS_OK;
}

void actuator_device_close(void)
{
    if (dshot)
    {
        (void)rt_device_close(dshot);
        dshot = NULL;
    }
    pending_apply = 0u;
}

void actuator_device_inhibit(void)
{
    if (dshot)
    {
        (void)rt_device_control(dshot, DSHOT_CTRL_STOP, NULL);
    }
}

uint32_t actuator_device_faults(void)
{
    struct dshot_stats stats;
    return dshot && rt_device_control(dshot, DSHOT_CTRL_GET_STATS, &stats) == RT_EOK ?
           stats.faults : 0u;
}

Status actuator_device_commit(const uint16_t commands[4], uint64_t *actual_apply_us)
{
    uint16_t packets[4];
    struct dshot_stats before, submitted;
    if (!dshot || !commands || !actual_apply_us || pending_apply)
        return STATUS_NOT_READY;
    if (rt_device_control(dshot, DSHOT_CTRL_GET_STATS, &before) != RT_EOK)
        return STATUS_NOT_READY;
    for (unsigned channel = 0; channel < 4; ++channel)
        packets[channel] = dshot_encode_packet(commands[channel], 0u);
    if (rt_device_write(dshot, 0, packets, sizeof(packets)) != (rt_ssize_t)sizeof(packets))
        return STATUS_NOT_READY;
    if (rt_device_control(dshot, DSHOT_CTRL_GET_STATS, &submitted) != RT_EOK ||
        submitted.frames != before.frames + 1u || submitted.faults != before.faults)
        return STATUS_NOT_READY;

    /* rt_device_write only starts DMA. The M7 guard must wait for the frame-complete ISR. */
    pending_frame_count = submitted.frames;
    pending_fault_count = submitted.faults;
    pending_apply = 1u;
    *actual_apply_us = 0u;
    return STATUS_OK;
}

Status actuator_device_poll_apply(uint64_t *actual_apply_us)
{
    struct dshot_stats stats;
    if (!dshot || !actual_apply_us || !pending_apply)
        return STATUS_NOT_READY;
    if (rt_device_control(dshot, DSHOT_CTRL_GET_STATS, &stats) != RT_EOK) {
        pending_apply = 0u;
        return STATUS_DEADLINE_MISSED;
    }
    if (stats.faults != pending_fault_count) {
        pending_apply = 0u;
        return STATUS_NOT_READY; /* M7 fault_count observes and latches the driver fault. */
    }
    if (stats.completed_frames < pending_frame_count) {
        if (!stats.active && !stats.pending) {
            pending_apply = 0u;
            return STATUS_DEADLINE_MISSED;
        }
        return STATUS_NOT_READY;
    }
    pending_apply = 0u;
    /* Timestamp after the ISR has stopped the waveform: conservative apply upper bound. */
    return clock_now_us(actual_apply_us);
}
