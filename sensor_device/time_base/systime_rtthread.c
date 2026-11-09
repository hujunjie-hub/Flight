#include "timer_capture.h"
#include "board_identity.h"
#include "systime.h"
#include "ports/port_api.h"
#include <string.h>

static struct rt_mutex g_time_mutex;
static struct rt_thread g_time_thread;
static uint8_t g_time_stack[1536] __attribute__((aligned(RT_ALIGN_SIZE)));
static rt_device_t g_timebase;

static void raw_lock(void *ctx)
{
    (void)rt_mutex_take((rt_mutex_t)ctx, RT_WAITING_FOREVER);
}

static void raw_unlock(void *ctx)
{
    (void)rt_mutex_release((rt_mutex_t)ctx);
}

static Status ticks(uint64_t *value, uint32_t *epoch)
{
    struct timebase_instant instant;
    if (!g_timebase ||
        rt_device_read(g_timebase, 0, &instant, sizeof(instant)) != (rt_ssize_t)sizeof(instant))
    {
        return STATUS_NOT_READY;
    }
    *value = instant.ticks;
    *epoch = instant.epoch;
    return STATUS_OK;
}

static void time_worker(void *parameter)
{
    (void)parameter;
    for (;;)
    {
        struct timebase_capture cap;
        while (rt_device_control(g_timebase, TIMEBASE_CTRL_READ_CAPTURE, &cap) == RT_EOK)
        {
            PpsCaptureEvent event;
            memset(&event, 0, sizeof(event));
            event.capture_tick64 = cap.tick64;
            event.time_epoch = 1u;
            event.edge_sequence = cap.sequence;
            event.capture_flags = (cap.flags & 0x02u) ? TIMER_CAPTURE_OVERCAPTURE
                                                      : TIMER_CAPTURE_VALID;
            (void)time_submit_pps(&event);
        }
        {
            TimeServiceStatus status;
            (void)time_get_status(&status);
        }
        rt_thread_mdelay(100);
    }
}

static int board_systime_init(void)
{
    TimeConfig cfg;
    uint32_t tick_hz = 0u;

    g_timebase = rt_device_find(TIMEBASE_DEVICE_NAME);
    if (!g_timebase || rt_device_open(g_timebase, RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
    {
        return -RT_ERROR;
    }
    if (rt_device_control(g_timebase, TIMEBASE_CTRL_GET_TICK_HZ, &tick_hz) != RT_EOK ||
        !tick_hz)
    {
        goto close_timebase;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.boot_id = board_boot_id();
    cfg.time_epoch = 1u;
    cfg.tick_hz = tick_hz;
    cfg.holdover_rate_bound_ppb = 500000u;
    cfg.gnss_pps_installed = 1u;

    if (rt_mutex_init(&g_time_mutex, "time", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        goto close_timebase;
    }
    systime_set_access_hooks(raw_lock, raw_unlock, &g_time_mutex);
    if (systime_init(&cfg, ticks) != STATUS_OK)
    {
        goto clear_hooks;
    }

    if (rt_thread_init(&g_time_thread, "time", time_worker, 0, g_time_stack,
                       sizeof(g_time_stack), 14u, 1u) != RT_EOK)
    {
        goto stop_service;
    }
    if (rt_thread_startup(&g_time_thread) == RT_EOK)
    {
        return RT_EOK;
    }
    (void)rt_thread_detach(&g_time_thread);

stop_service:
    (void)systime_stop();
clear_hooks:
    systime_set_access_hooks(NULL, NULL, NULL);
    (void)rt_mutex_detach(&g_time_mutex);
close_timebase:
    (void)rt_device_close(g_timebase);
    g_timebase = NULL;
    return -RT_ERROR;
}
/* Boot identity is captured by the component init stage. */
INIT_ENV_EXPORT(board_systime_init);
