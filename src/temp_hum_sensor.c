#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <stdbool.h>

#include "temp_hum_sensor.h"
#include "sensor_messages.h"

/* Semaphore synchronizing the sampling timer with the thread */
K_SEM_DEFINE(temp_hum_sample_sem, 0, 1);

#define I2C_NODE DT_ALIAS(sensor_i2c)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
#define SI7021_ADDR 0x40

/* Si7021 I2C command codes */
#define SI7021_CMD_MEAS_HOLD_TEMP 0xE3
#define SI7021_CMD_MEAS_HOLD_HUM  0xE5

static void temp_hum_timer_handler(struct k_timer *timer_id)
{
    k_sem_give(&temp_hum_sample_sem);
}

K_TIMER_DEFINE(temp_hum_timer, temp_hum_timer_handler, NULL);

/* Send Si7021 fault status to the central message queue */
static void temp_hum_report_fault(bool active, int error_code)
{
    struct sensor_message msg = {
        .type = MSG_SENSOR_FAULT,
        .data.fault = {
            .sensor = SENSOR_ENV,
            .active = active,
            .error_code = error_code
        }
    };

    k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT);
}



void temp_hum_thread_entry(void *p1, void *p2, void *p3)
{
    bool fault_active = false;


    if (!device_is_ready(i2c_dev)) {
        printk("[Error]: I2C bus not ready for Si7021\n");
        temp_hum_report_fault(true, -1);
        return;
    }

    k_timer_start(&temp_hum_timer, K_NO_WAIT, K_SECONDS(2));

    while (1) {
        k_sem_take(&temp_hum_sample_sem, K_FOREVER);

        uint8_t rx_buf[2];
        struct env_data current_data = {0};

        /* Read Temperature */
        uint8_t cmd_temp = SI7021_CMD_MEAS_HOLD_TEMP;
        if (i2c_write_read(i2c_dev, SI7021_ADDR, &cmd_temp, 1, rx_buf, 2) == 0) {
            uint16_t raw_temp = (rx_buf[0] << 8) | rx_buf[1];
            /* Conversion formula from Si7021 datasheet: T = (175.72 * raw / 65536) - 46.85 */
            current_data.temperature = (175.72f * (float)raw_temp / 65536.0f) - 46.85f;
        } else {
            printk("[Error]: Failed to read Si7021 temperature\n");

            if (!fault_active) {
                temp_hum_report_fault(true, -1);
                fault_active = true;
            }

            continue;
        }

        /* Read Relative Humidity */
        uint8_t cmd_hum = SI7021_CMD_MEAS_HOLD_HUM;
        if (i2c_write_read(i2c_dev, SI7021_ADDR, &cmd_hum, 1, rx_buf, 2) == 0) {

            /* Report recovery after a previous reading error */
            if (fault_active) {
                temp_hum_report_fault(false, 0);
                fault_active = false;
            }

            uint16_t raw_hum = (rx_buf[0] << 8) | rx_buf[1];
            /* Conversion formula from Si7021 datasheet: RH = (125 * raw / 65536) - 6 */
            current_data.humidity = (125.0f * (float)raw_hum / 65536.0f) - 6.0f;

            /* Clamp humidity within 0-100% range */
            if (current_data.humidity < 0.0f) {
                current_data.humidity = 0.0f;
            }

            if (current_data.humidity > 100.0f) {
                current_data.humidity = 100.0f;
            }

            /* Send environmental data to the central thread */
            struct sensor_message msg = {
                .type = MSG_SENSOR_ENV,
                .data.env = current_data
            };
            k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT);
        } else {
            printk("[Error]: Failed to read Si7021 humidity\n");

            if (!fault_active) {
                temp_hum_report_fault(true, -1);
                fault_active = true;
            }
        }
    }
}

K_THREAD_DEFINE(temp_hum_thread_id, 1024, temp_hum_thread_entry, NULL, NULL, NULL, 7, 0, 0);