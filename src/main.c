#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include "sensor_messages.h"

/* Central message queue definition */
K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_message), 10, 4);

/* B1 button configuration */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb;

/* Tracker modes */
enum tracker_mode { MODE_TEST, MODE_NORMAL, MODE_SIMU };
static enum tracker_mode current_mode = MODE_TEST;

/* Button interrupt callback */
static void button_pressed(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    static int64_t last_press_ms;
    int64_t now = k_uptime_get();
    
    /* 200ms debounce */
    if (now - last_press_ms < 200) return;
    last_press_ms = now;

    /* Non-blocking send of the button event to the central queue */
    struct sensor_message msg = { .type = MSG_BUTTON_PRESSED };
    k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT);
}

int main(void)
{
    printk("[INFO] Main thread started - MSGQ initialized\n");

    /* Initialize B1 button GPIO */
    if (gpio_is_ready_dt(&button)) {
        gpio_pin_configure_dt(&button, GPIO_INPUT);
        gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
        gpio_add_callback(button.port, &button_cb);
        gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    }
    
    printk("[MODE] TEST\n");

    struct sensor_message msg;

    while (1) {
        /* Wait forever for any sensor or button to put data in the queue */
        if (k_msgq_get(&sensor_msgq, &msg, K_FOREVER) == 0) {
            
            switch (msg.type) {
                case MSG_BUTTON_PRESSED:
                    /* Mode transition logic: TEST -> NORMAL -> SIMU -> TEST */
                    if (current_mode == MODE_TEST) {
                        current_mode = MODE_NORMAL;
                    } else if (current_mode == MODE_NORMAL) {
                        current_mode = MODE_SIMU;
                    } else {
                        current_mode = MODE_TEST;
                    }
                    
                    printk("[MODE] %s\n", 
                           current_mode == MODE_TEST ? "TEST" : 
                          (current_mode == MODE_NORMAL ? "NORMAL" : "SIMU"));
                    break;

                case MSG_SENSOR_LDR:
                    printk("[MSGQ LDR] Raw: %u | Brightness: %u%%\n", 
                           msg.data.ldr.raw, 
                           msg.data.ldr.brightness);
                    break;

                case MSG_SENSOR_MPU:
                    printk("[MSGQ MPU] Accel (m/s2): X=%.2f Y=%.2f Z=%.2f | Gyro (deg/s): X=%.2f Y=%.2f Z=%.2f\n", 
                           (double)msg.data.imu.measurements.ax, 
                           (double)msg.data.imu.measurements.ay, 
                           (double)msg.data.imu.measurements.az,
                           (double)msg.data.imu.measurements.gx, 
                           (double)msg.data.imu.measurements.gy, 
                           (double)msg.data.imu.measurements.gz);
                    printk("[MSGQ MPU] Attitude (deg): Roll=%.2f Pitch=%.2f Yaw=%.2f\n",
                           (double)msg.data.imu.roll, 
                           (double)msg.data.imu.pitch, 
                           (double)msg.data.imu.yaw);
                    break;

                case MSG_SENSOR_GPS:
                    if (msg.data.gps.fix_valid) {
                        printk("[MSGQ GPS] Fix OK | UTC: %s | Sats: %u | Lat: %.6f | Lon: %.6f | Alt: %.1f m\n", 
                               msg.data.gps.utc,
                               msg.data.gps.satellites,
                               msg.data.gps.latitude, 
                               msg.data.gps.longitude,
                               (double)msg.data.gps.altitude);
                    } else {
                        printk("[MSGQ GPS] Waiting for 3D fix...\n");
                    }
                    break;

                case MSG_SENSOR_ENV:
                    printk("[MSGQ ENV] Temp: %.2f C | Hum: %.2f %%\n", 
                           (double)msg.data.env.temperature, 
                           (double)msg.data.env.humidity);
                    break;
                    
                default:
                    printk("[MSGQ] Unknown message type received\n");
                    break;
            }
        }
    }
    return 0;
}