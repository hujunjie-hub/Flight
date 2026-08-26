/*
 * Flight Controller - Main Application
 *
 * ADIS16505 IMU Test: Read IMU data and output via USART1 (Debug Serial)
 *
 * Hardware:
 *   USART1 (PA9=TX, PA10=RX) - Debug Serial Console
 *   ADIS16505 (SPI1) - IMU Sensor
 *   LED PG7 - Status LED
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define LED_PIN     GET_PIN(G, 7)

/* ADIS16505 Data Structure (must match sensor driver) */
struct adis16505_data
{
    rt_int16_t x_gyro;
    rt_int16_t y_gyro;
    rt_int16_t z_gyro;
    rt_int16_t x_accl;
    rt_int16_t y_accl;
    rt_int16_t z_accl;
    rt_int16_t x_magn;
    rt_int16_t y_magn;
    rt_int16_t z_magn;
    rt_int16_t temperature;
    rt_uint32_t timer;
    rt_uint32_t timestamp;
};

/* External function from ADIS16505 driver */
extern rt_err_t adis16505_get_data(struct adis16505_data *data);

int main(void)
{
    struct adis16505_data imu_data;
    rt_err_t ret;
    rt_uint32_t count = 0;

    /* Configure LED pin */
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);

    rt_kprintf("\n========================================\n");
    rt_kprintf("  Flight Controller - IMU Test\n");
    rt_kprintf("  USART1 Debug Console: 115200 8N1\n");
    rt_kprintf("========================================\n\n");

    /* Wait for ADIS16505 sensor initialization */
    rt_thread_mdelay(1000);

    while (1)
    {
        /* Get IMU data from ADIS16505 driver */
        ret = adis16505_get_data(&imu_data);

        if (ret == RT_EOK)
        {
            /* Toggle LED to indicate data is being read */
            rt_pin_write(LED_PIN, (count % 2) ? PIN_HIGH : PIN_LOW);

            /* Output IMU data via USART1 */
            rt_kprintf("[%06lu] IMU Data:\n", count);
            rt_kprintf("  Gyro: X=%+6d Y=%+6d Z=%+6d (raw)\n",
                       imu_data.x_gyro, imu_data.y_gyro, imu_data.z_gyro);
            rt_kprintf("  Accl: X=%+6d Y=%+6d Z=%+6d (raw)\n",
                       imu_data.x_accl, imu_data.y_accl, imu_data.z_accl);
            rt_kprintf("  Magn: X=%+6d Y=%+6d Z=%+6d (raw)\n",
                       imu_data.x_magn, imu_data.y_magn, imu_data.z_magn);
            rt_kprintf("  Temp: %+6d (raw)\n", imu_data.temperature);
            rt_kprintf("\n");
        }
        else
        {
            rt_kprintf("[%06lu] IMU read error: %d\n", count, ret);
        }

        count++;

        /* 10Hz output rate for testing */
        rt_thread_mdelay(100);
    }

    return RT_EOK;
}
