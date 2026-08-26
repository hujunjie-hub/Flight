/*
 * UM982 RTK GNSS Driver
 *
 * Hardware Interface:
 *   USART2 (PA2=TXD/STM32_TX, PA3=RXD/STM32_RX, DMA1_Stream2 RX Circular)
 *   TIM2_CH1 PA0 (1PPS time pulse)
 *
 * Configuration:
 *   Baud Rate: 115200 (default)
 *   Output Rate: 10Hz
 *   Protocol: NMEA / RTCM
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define UM982_UART_DEVICE       "uart2"
#define UM982_PPS_PIN           GET_PIN(A, 0)

#define UM982_RX_BUF_SIZE       1024
#define UM982_TX_BUF_SIZE       256
#define UM982_NMEA_MAX_LEN      256

/* GNSS Fix Quality */
#define UM982_FIX_NONE          0
#define UM982_FIX_2D            1
#define UM982_FIX_3D            2
#define UM982_FIX_RTK_FLOAT     4
#define UM982_FIX_RTK_FIXED     5

/* GNSS Data Structure */
struct um982_gnss_data
{
    /* Time */
    rt_uint8_t  hours;
    rt_uint8_t  minutes;
    rt_uint8_t  seconds;
    rt_uint16_t milliseconds;

    /* Position */
    rt_double_t latitude;       /* degrees */
    rt_double_t longitude;      /* degrees */
    rt_double_t altitude;       /* meters (ellipsoid) */
    rt_double_t altitude_msl;   /* meters (mean sea level) */

    /* Velocity */
    rt_double_t ground_speed;   /* knots */
    rt_double_t heading;        /* degrees */

    /* Quality */
    rt_uint8_t  fix_type;
    rt_uint8_t  satellites_used;
    rt_double_t hdop;
    rt_double_t vdop;
    rt_double_t pdop;

    /* DOPS */
    rt_double_t elevation[12];
    rt_double_t azimuth[12];
    rt_int32_t  snr[12];

    /* Status */
    rt_bool_t   valid;
    rt_bool_t   pps_sync;
    rt_uint32_t pps_timestamp;
    rt_uint32_t timestamp;

    /* Raw */
    rt_uint8_t  nmea_buf[UM982_NMEA_MAX_LEN];
    rt_uint16_t nmea_len;
};

struct um982_device
{
    rt_device_t  uart_dev;
    struct um982_gnss_data data;
    rt_sem_t     rx_sem;
    rt_sem_t     pps_sem;
    rt_mutex_t   lock;
    rt_uint8_t   rx_buf[UM982_RX_BUF_SIZE];
    rt_uint8_t   tx_buf[UM982_TX_BUF_SIZE];
    rt_uint16_t  rx_cnt;
    rt_bool_t    nmea_ready;
};

static struct um982_device um982_dev = {0};

/* NMEA Parser: $GPGGA or $GNGGA */
static rt_err_t um982_parse_gga(const char *nmea, struct um982_gnss_data *data)
{
    const char *p = nmea;
    char *token;
    char field[16];
    int field_idx = 0;

    /* Skip talker ID ($GP or $GN) and message ID (GGA) */
    p = strchr(p, ',');
    if (p == RT_NULL) return -RT_ERROR;
    p++;

    /* Time (hhmmss.ss) */
    token = strsep((char **)&p, ",");
    if (token && strlen(token) >= 6)
    {
        data->hours   = (token[0] - '0') * 10 + (token[1] - '0');
        data->minutes = (token[2] - '0') * 10 + (token[3] - '0');
        data->seconds = (token[4] - '0') * 10 + (token[5] - '0');
        if (strlen(token) > 7)
            data->milliseconds = (token[7] - '0') * 100;
    }

    /* Latitude (ddmm.mmmm) */
    token = strsep((char **)&p, ",");
    if (token && strlen(token) > 0)
    {
        rt_double_t lat = atof(token);
        int deg = (int)(lat / 100);
        rt_double_t min = lat - deg * 100;
        data->latitude = deg + min / 60.0;
    }

    /* N/S */
    token = strsep((char **)&p, ",");
    if (token && token[0] == 'S')
        data->latitude = -data->latitude;

    /* Longitude (dddmm.mmmm) */
    token = strsep((char **)&p, ",");
    if (token && strlen(token) > 0)
    {
        rt_double_t lon = atof(token);
        int deg = (int)(lon / 100);
        rt_double_t min = lon - deg * 100;
        data->longitude = deg + min / 60.0;
    }

    /* E/W */
    token = strsep((char **)&p, ",");
    if (token && token[0] == 'W')
        data->longitude = -data->longitude;

    /* Fix quality */
    token = strsep((char **)&p, ",");
    if (token)
        data->fix_type = atoi(token);

    /* Satellites used */
    token = strsep((char **)&p, ",");
    if (token)
        data->satellites_used = atoi(token);

    /* HDOP */
    token = strsep((char **)&p, ",");
    if (token && strlen(token) > 0)
        data->hdop = atof(token);

    /* Altitude (MSL) */
    token = strsep((char **)&p, ",");
    if (token && strlen(token) > 0)
        data->altitude_msl = atof(token);

    data->valid = (data->fix_type >= 1) ? RT_TRUE : RT_FALSE;

    return RT_EOK;
}

/* NMEA Parser: $GPRMC or $GNRMC */
static rt_err_t um982_parse_rmc(const char *nmea, struct um982_gnss_data *data)
{
    const char *p = nmea;
    char *token;

    /* Skip to first field */
    p = strchr(p, ',');
    if (p == RT_NULL) return -RT_ERROR;
    p++;

    /* Time */
    token = strsep((char **)&p, ",");
    if (token && strlen(token) >= 6)
    {
        data->hours   = (token[0] - '0') * 10 + (token[1] - '0');
        data->minutes = (token[2] - '0') * 10 + (token[3] - '0');
        data->seconds = (token[4] - '0') * 10 + (token[5] - '0');
    }

    /* Status (A=active, V=void) */
    token = strsep((char **)&p, ",");
    if (token && token[0] == 'A')
        data->valid = RT_TRUE;

    /* Skip latitude, N/S, longitude, E/W (already parsed in GGA) */
    token = strsep((char **)&p, ","); /* lat */
    token = strsep((char **)&p, ","); /* N/S */
    token = strsep((char **)&p, ","); /* lon */
    token = strsep((char **)&p, ","); /* E/W */

    /* Ground speed (knots) */
    token = strsep((char **)&p, ",");
    if (token && strlen(token) > 0)
        data->ground_speed = atof(token);

    /* Heading */
    token = strsep((char **)&p, ",");
    if (token && strlen(token) > 0)
        data->heading = atof(token);

    return RT_EOK;
}

/* Process received NMEA sentence */
static void um982_process_nmea(const char *nmea, rt_size_t len)
{
    if (len < 6) return;

    /* Check for GGA sentence */
    if (strstr(nmea, "$G") != RT_NULL && strstr(nmea, "GGA,") != RT_NULL)
    {
        um982_parse_gga(nmea, &um982_dev.data);
    }
    /* Check for RMC sentence */
    else if (strstr(nmea, "$G") != RT_NULL && strstr(nmea, "RMC,") != RT_NULL)
    {
        um982_parse_rmc(nmea, &um982_dev.data);
    }

    um982_dev.data.timestamp = rt_tick_get();
}

/* UART RX callback */
static rt_err_t um982_uart_rx_indicate(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(um982_dev.rx_sem);
    return RT_EOK;
}

/* PPS interrupt callback */
static void um982_pps_irq_callback(void *args)
{
    um982_dev.data.pps_timestamp = rt_tick_get();
    um982_dev.data.pps_sync = RT_TRUE;
    rt_sem_release(um982_dev.pps_sem);
}

/* Send command to UM982 */
static rt_err_t um982_send_cmd(const char *cmd, rt_size_t len)
{
    if (um982_dev.uart_dev == RT_NULL)
        return -RT_ERROR;

    rt_device_write(um982_dev.uart_dev, 0, cmd, len);

    return RT_EOK;
}

/* UM982 Device Init */
static rt_err_t um982_init(void)
{
    rt_err_t ret;
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    /* Find UART device */
    um982_dev.uart_dev = rt_device_find(UM982_UART_DEVICE);
    if (um982_dev.uart_dev == RT_NULL)
    {
        rt_kprintf("[UM982] UART device not found: %s\n", UM982_UART_DEVICE);
        return -RT_ERROR;
    }

    /* Configure UART: 115200, 8N1 */
    cfg.baud_rate = BAUD_RATE_115200;
    cfg.bufsz = UM982_RX_BUF_SIZE;
    ret = rt_device_control(um982_dev.uart_dev, RT_DEVICE_CTRL_CONFIG, &cfg);
    if (ret != RT_EOK)
    {
        rt_kprintf("[UM982] UART configure failed: %d\n", ret);
        return ret;
    }

    /* Open UART with interrupt RX mode */
    ret = rt_device_open(um982_dev.uart_dev, RT_DEVICE_FLAG_INT_RX);
    if (ret != RT_EOK)
    {
        rt_kprintf("[UM982] UART open failed: %d\n", ret);
        return ret;
    }

    /* Set RX indicate callback */
    rt_device_set_rx_indicate(um982_dev.uart_dev, um982_uart_rx_indicate);

    /* Create semaphores */
    um982_dev.rx_sem = rt_sem_create("um982_rx", 0, RT_IPC_FLAG_FIFO);
    um982_dev.pps_sem = rt_sem_create("um982_pps", 0, RT_IPC_FLAG_FIFO);
    if (um982_dev.rx_sem == RT_NULL || um982_dev.pps_sem == RT_NULL)
    {
        rt_kprintf("[UM982] Semaphore create failed\n");
        return -RT_ERROR;
    }

    /* Configure PPS pin as input */
    rt_pin_mode(UM982_PPS_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(UM982_PPS_PIN, PIN_IRQ_MODE_RISING, um982_pps_irq_callback, RT_NULL);
    rt_pin_irq_enable(UM982_PPS_PIN, PIN_IRQ_ENABLE);

    rt_kprintf("[UM982] Init OK\n");

    return RT_EOK;
}

/* GNSS data reading thread */
static void um982_thread_entry(void *parameter)
{
    rt_uint8_t ch;
    char nmea_buf[UM982_NMEA_MAX_LEN];
    rt_uint16_t nmea_idx = 0;

    rt_kprintf("[UM982] Thread started\n");

    while (1)
    {
        /* Wait for UART RX data */
        rt_sem_recv(um982_dev.rx_sem, 1, RT_WAITING_FOREVER);

        while (rt_device_read(um982_dev.uart_dev, -1, &ch, 1) == 1)
        {
            if (ch == '$')
            {
                /* Start of NMEA sentence */
                nmea_idx = 0;
                nmea_buf[nmea_idx++] = ch;
            }
            else if (nmea_idx > 0)
            {
                nmea_buf[nmea_idx++] = ch;

                if (ch == '\n' || nmea_idx >= UM982_NMEA_MAX_LEN - 1)
                {
                    /* End of NMEA sentence */
                    nmea_buf[nmea_idx] = '\0';

                    /* Process complete NMEA sentence */
                    rt_mutex_take(um982_dev.lock, RT_WAITING_FOREVER);
                    um982_process_nmea(nmea_buf, nmea_idx);
                    rt_mutex_release(um982_dev.lock);

                    nmea_idx = 0;
                }
            }
        }
    }
}

/* Get latest GNSS data */
rt_err_t um982_get_data(struct um982_gnss_data *data)
{
    if (data == RT_NULL)
        return -RT_ERROR;

    rt_mutex_take(um982_dev.lock, RT_WAITING_FOREVER);
    *data = um982_dev.data;
    rt_mutex_release(um982_dev.lock);

    return RT_EOK;
}

/* Register GNSS driver */
static int um982_register(void)
{
    rt_err_t ret;
    rt_thread_t thread;

    ret = um982_init();
    if (ret != RT_EOK)
    {
        rt_kprintf("[UM982] Init failed: %d\n", ret);
        return ret;
    }

    /* Create GNSS data reading thread */
    thread = rt_thread_create("um982_read", um982_thread_entry, RT_NULL,
                              4096, RT_THREAD_PRIORITY_MAX - 4, 10);
    if (thread == RT_NULL)
    {
        rt_kprintf("[UM982] Thread create failed\n");
        return -RT_ERROR;
    }

    rt_thread_startup(thread);

    return RT_EOK;
}

INIT_APP_EXPORT(um982_register);
