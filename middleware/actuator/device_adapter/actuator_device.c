#include "device_adapter/actuator_device.h"
#include "dshot.h"
#include "dshot_packet.h"
#include "ports/port_api.h"

static rt_device_t dshot;

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
    if (!dshot || !commands || !actual_apply_us)
    {
        return STATUS_NOT_READY;
    }
    for (unsigned channel = 0; channel < 4; ++channel)
    {
        packets[channel] = dshot_encode_packet(commands[channel], 0u);
    }
    if (rt_device_write(dshot, 0, packets, sizeof(packets)) != (rt_ssize_t)sizeof(packets))
    {
        return STATUS_NOT_READY;
    }
    return clock_now_us(actual_apply_us);
}
