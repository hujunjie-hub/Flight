/* FinSH diagnostics and console selection for this board. */
#include "adis16505.h"
#include "bmm350.h"
#include "bmp585.h"
#include "dshot.h"
#include "timer_capture.h"
#include "board_identity.h"
#include "drv_usart_dma.h"

#ifdef RT_USING_FINSH
#include <finsh.h>

static void driver_status(void)
{
    rt_device_t dev;
    struct imu_stats imu;
    struct mag_stats mag;
    struct barometer_stats baro;
    struct timebase_stats tb;
    struct dshot_stats dshot;
    UartStats radio;
    UartStats rc;
    UartStats gnss;

    rt_kprintf("Flight driver devices (boot=%08x%08x)\n",
               (uint32_t)(board_boot_id() >> 32), (uint32_t)board_boot_id());

    dev = rt_device_find(TIMEBASE_DEVICE_NAME);
    if (dev && rt_device_control(dev, TIMEBASE_CTRL_GET_STATS, &tb) == RT_EOK)
    {
        rt_kprintf("timebase: tick=%u Hz lost=%u imu_capture=%u\n",
                   tb.tick_hz, tb.capture_lost, tb.imu_capture_enabled);
    }
    else
    {
        rt_kprintf("timebase: not available\n");
    }

    dev = rt_device_find(IMU_DEVICE_NAME);
    if (dev && rt_device_control(dev, IMU_CTRL_GET_STATS, &imu) == RT_EOK)
    {
        rt_kprintf("imu: received=%u dropped=%u errors=%u checksum=%u ring_drop=%u\n",
                   imu.received, imu.dropped, imu.errors, imu.checksum_errors,
                   imu.raw_dropped);
    }
    else
    {
        rt_kprintf("imu: not available\n");
    }

    dev = rt_device_find(MAG_DEVICE_NAME);
    if (dev && rt_device_control(dev, MAG_CTRL_GET_STATS, &mag) == RT_EOK)
    {
        rt_kprintf("mag_sensor: samples=%u errors=%u\n", mag.samples, mag.errors);
    }
    else
    {
        rt_kprintf("mag_sensor: not available\n");
    }

    dev = rt_device_find(BAROMETER_DEVICE_NAME);
    if (dev && rt_device_control(dev, BAROMETER_CTRL_GET_STATS, &baro) == RT_EOK)
    {
        rt_kprintf("barometer: samples=%u errors=%u\n", baro.samples, baro.errors);
    }
    else
    {
        rt_kprintf("barometer: not available\n");
    }

    dev = rt_device_find(DSHOT_DEVICE_NAME);
    if (dev && rt_device_control(dev, DSHOT_CTRL_GET_STATS, &dshot) == RT_EOK)
    {
        rt_kprintf("dshot: frames=%u faults=%u arr=%u active=%u pending=%u\n",
                   dshot.frames, dshot.faults, dshot.arr, dshot.active, dshot.pending);
    }
    else
    {
        rt_kprintf("dshot: not available\n");
    }

    if (uart_get_stats(USART2, &radio) == RT_EOK &&
        uart_get_stats(USART3, &rc) == RT_EOK &&
        uart_get_stats(UART4, &gnss) == RT_EOK)
    {
        rt_kprintf("uart2 rx=%u lost=%u err=%u; uart3 rx=%u lost=%u err=%u; uart4 rx=%u lost=%u err=%u\n",
                   radio.rx_bytes, radio.overwritten_bytes, radio.error_events,
                   rc.rx_bytes, rc.overwritten_bytes, rc.error_events,
                   gnss.rx_bytes, gnss.overwritten_bytes, gnss.error_events);
    }
}
MSH_CMD_EXPORT(driver_status, Show Flight driver status);

#if defined(BSP_USING_USBD) && defined(RT_USB_DEVICE_CDC)
#include <rtdevice.h>
#include <shell.h>
#include <string.h>

static int flight_console(int argc, char **argv)
{
    const char *name;
    if (argc == 2 && strcmp(argv[1], "usb") == 0)
    {
        rt_device_t usb = rt_device_find("vcom");
        rt_bool_t connected = RT_FALSE;
        if (!usb || rt_device_control(usb, RT_USBD_CLASS_CTRL_CONNECTED, &connected) != RT_EOK ||
            !connected)
        {
            rt_kprintf("Open the USB virtual serial port first.\n");
            return -RT_ERROR;
        }
        name = "vcom";
    }
    else if (argc == 2 && strcmp(argv[1], "uart") == 0)
    {
        name = RT_CONSOLE_DEVICE_NAME;
    }
    else
    {
        rt_kprintf("flight_console usb|uart\n");
        return -RT_EINVAL;
    }
    finsh_set_device(name);
    const char *active = finsh_get_device();
    if (active && strcmp(active, name) == 0)
    {
        rt_console_set_device(name);
        rt_kprintf("Flight console on %s\n", name);
        return RT_EOK;
    }
    return -RT_EIO;
}
MSH_CMD_EXPORT(flight_console, Select the USB or UART maintenance console);
#endif
#endif
