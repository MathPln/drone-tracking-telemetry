#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

/* Semaphore for synchronization between the hardware timer and the thread */
K_SEM_DEFINE(ldr_sample_sem, 0, 1);

/* ADC channel definition from the devicetree */
#define ADC_NODE DT_PATH(zephyr_user)
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(ADC_NODE);

/* Timer callback: simply gives the semaphore to unblock the thread */
void ldr_timer_handler(struct k_timer *timer_id)
{
    k_sem_give(&ldr_sample_sem);
}

K_TIMER_DEFINE(ldr_timer, ldr_timer_handler, NULL);

void ldr_thread_entry(void *p1, void *p2, void *p3)
{
    if (!adc_is_ready_dt(&adc_channel)) {
        printk("[Error]: ADC not ready\n");
        return;
    }

    int err = adc_channel_setup_dt(&adc_channel);
    if (err < 0) {
        printk("[Error]: ADC setup failed with code %d\n", err);
        return;
    }

    int16_t sample_buffer;
    struct adc_sequence sequence = {
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
    };

    /* Start the timer */
    k_timer_start(&ldr_timer, K_NO_WAIT, K_SECONDS(2));

    while (1) {
        /* Wait for the timer tick via the semaphore */
        k_sem_take(&ldr_sample_sem, K_FOREVER);

        adc_sequence_init_dt(&adc_channel, &sequence);
        err = adc_read_dt(&adc_channel, &sequence);

        if (err == 0) {
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

            printk("[Info]: LDR ADC raw: %d | Brightness: %d%%\n", (int)raw, percent);
        } else {
            printk("[Error]: ADC read failed with code %d\n", err);
        }
    }
}

/* Start the thread automatically */
K_THREAD_DEFINE(ldr_thread_id, 1024, ldr_thread_entry, NULL, NULL, NULL, 7, 0, 0);