#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <stdbool.h>

#include "temp_hum_sensor.h"
#include "sensor_messages.h"

/* LDR and Si7021 share a 2-second acquisition cycle. */
K_SEM_DEFINE(environment_sample_sem, 0, 1);

#define ADC_NODE DT_PATH(zephyr_user)
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(ADC_NODE);

#define I2C_NODE DT_ALIAS(sensor_i2c)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
#define SI7021_ADDR 0x40

/* Si7021 I2C command codes */
#define SI7021_CMD_MEAS_HOLD_TEMP 0xE3
#define SI7021_CMD_MEAS_HOLD_HUM  0xE5

static void environment_timer_handler(struct k_timer *timer_id)
{
    k_sem_give(&environment_sample_sem);
}

K_TIMER_DEFINE(environment_sample_timer, environment_timer_handler, NULL);

static void ldr_report_fault(bool active, int error_code)
{
    struct sensor_message msg = {
        .type = MSG_SENSOR_FAULT,
        .data.fault = {
            .sensor = SENSOR_LDR,
            .active = active,
            .error_code = error_code
        }
    };

    k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT);
}

/* Send Si7021 fault status to the central message queue */
static void environment_report_fault(bool active, int error_code)
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



void environment_thread_entry(void *p1, void *p2, void *p3)
{
    bool ldr_fault_active = false;
    bool env_fault_active = false;
    bool adc_ready = false;
    bool i2c_ready = device_is_ready(i2c_dev);
    int err;

    if (!adc_is_ready_dt(&adc_channel)) {
        if (!simu_mode_active) {
        printk("[Error]: LDR ADC not ready\n");
        }
        ldr_report_fault(true, -1);
        ldr_fault_active = true;
    } else {
        err = adc_channel_setup_dt(&adc_channel);
        if (err < 0) {
            if (!simu_mode_active) {
            printk("[Error]: ADC setup failed with code %d\n", err);
            }
            ldr_report_fault(true, err);
            ldr_fault_active = true;
        } else {
            adc_ready = true;
        }
    }

    if (!i2c_ready) {
        if (!simu_mode_active) {
        printk("[Error]: I2C bus not ready for Si7021\n");
        }
        environment_report_fault(true, -1);
        env_fault_active = true;
    }

    int16_t sample_buffer;
    struct adc_sequence sequence = {
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
    };

    /* Start the timer */
    k_timer_start(&environment_sample_timer, K_NO_WAIT, K_SECONDS(2));

    while (1) {
        /* Wait for the timer tick via the semaphore */
        k_sem_take(&environment_sample_sem, K_FOREVER);

        if (adc_ready) {
            adc_sequence_init_dt(&adc_channel, &sequence);
            err = adc_read_dt(&adc_channel, &sequence);

            if (err == 0) {
                /* Report recovery after a previous reading error */
                if (ldr_fault_active) {
                    ldr_report_fault(false, 0);
                    ldr_fault_active = false;
                }

                int32_t raw = sample_buffer;

                /* Calculate the maximum value for the ADC resolution. */
                int32_t max_raw = ((int32_t)1 << adc_channel.resolution) - 1;

                /* Convert the raw measurement into an inverted percentage. */
                int percent = 100 - (raw * 100 / max_raw);

                /* Keep the percentage within the 0–100 range. */
                if (percent < 0) {
                    percent = 0;
                } else if (percent > 100) {
                    percent = 100;
                }

                /* Send LDR measurements to the central thread */
                struct sensor_message msg = {
                    .type = MSG_SENSOR_LDR,
                    .data.ldr.raw = (uint16_t)raw,
                    .data.ldr.brightness = (uint8_t)percent
                };
                k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT);
            } else {
                if (!simu_mode_active) {
                printk("[Error]: ADC read failed with code %d\n", err);
                }
                /* Report the error only once until recovery */
                if (!ldr_fault_active) {
                    ldr_report_fault(true, err);
                    ldr_fault_active = true;
                }
            }
        }

        if (!i2c_ready) {
            continue;
        }

        uint8_t rx_buf[2];
        struct env_data current_data = {0};
        bool env_read_ok = true;

        /* Read Temperature */
        uint8_t cmd_temp = SI7021_CMD_MEAS_HOLD_TEMP;
        if (i2c_write_read(i2c_dev, SI7021_ADDR, &cmd_temp, 1, rx_buf, 2) == 0) {
            uint16_t raw_temp = (rx_buf[0] << 8) | rx_buf[1];
            current_data.temperature = (175.72f * (float)raw_temp / 65536.0f) - 46.85f;
        } else {
            if (!simu_mode_active) {
            printk("[Error]: Failed to read Si7021 temperature\n");
            }
            env_read_ok = false;
        }

        /* Read Relative Humidity */
        uint8_t cmd_hum = SI7021_CMD_MEAS_HOLD_HUM;
        if (i2c_write_read(i2c_dev, SI7021_ADDR, &cmd_hum, 1, rx_buf, 2) == 0) {
            uint16_t raw_hum = (rx_buf[0] << 8) | rx_buf[1];
            current_data.humidity = (125.0f * (float)raw_hum / 65536.0f) - 6.0f;

            if (current_data.humidity < 0.0f) {
                current_data.humidity = 0.0f;
            }

            if (current_data.humidity > 100.0f) {
                current_data.humidity = 100.0f;
            }
        } else {
            if (!simu_mode_active) {
            printk("[Error]: Failed to read Si7021 humidity\n");
            }
            env_read_ok = false;
        }

        if (env_read_ok) {
            if (env_fault_active) {
                environment_report_fault(false, 0);
                env_fault_active = false;
            }

            struct sensor_message msg = {
                .type = MSG_SENSOR_ENV,
                .data.env = current_data
            };
            k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT);
        } else if (!env_fault_active) {
            environment_report_fault(true, -1);
            env_fault_active = true;
        }
    }
}

K_THREAD_DEFINE(environment_thread_id, 1024, environment_thread_entry, NULL, NULL, NULL, 7, 0, 0);