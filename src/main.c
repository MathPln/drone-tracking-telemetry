#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include "sensor_messages.h"
#include "statistics.h"

/* Central message queue definition */
K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_message), 10, 4);

/* B1 button and RGB LED configuration */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(DT_ALIAS(led_red), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led_green), gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(DT_ALIAS(led_blue), gpios);

static struct gpio_callback button_cb;
static bool rgb_ready;

/* Set RGB color using logical GPIO states */
static void set_color(int red, int green, int blue)
{
    gpio_pin_set_dt(&led_red, red);
    gpio_pin_set_dt(&led_green, green);
    gpio_pin_set_dt(&led_blue, blue);
}

/* Tracker modes */
enum tracker_mode { MODE_TEST, MODE_NORMAL, MODE_SIMU };
static enum tracker_mode current_mode = MODE_TEST;

/* Statistics collection period in NORMAL mode */
#define STATS_PERIOD_MS 30000

/* Cached sensor data for periodic printing in TEST mode */
static struct ldr_data last_ldr;
static struct imu_message last_imu;
static struct gps_data last_gps;
static struct env_data last_env;

/* Initialize RGB LED */
static void rgb_init(void)
{
    if (gpio_is_ready_dt(&led_red) && gpio_is_ready_dt(&led_green) && gpio_is_ready_dt(&led_blue)) {
        gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_INACTIVE);
        rgb_ready = true;
    } else {
        printk("[ERROR] RGB LED not ready\n");
    }
}

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

/* Timer for 2-second periodic report in TEST mode */
static void test_timer_handler(struct k_timer *timer_id)
{
    struct sensor_message msg = { .type = MSG_TIMER_2S };
    k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT);
}
K_TIMER_DEFINE(test_timer, test_timer_handler, NULL);

int main(void)
{
    printk("[INFO] Main thread started - MSGQ initialized\n");
    
    /* Initialize the statistics module and LEDs */
    statistics_init();
    rgb_init();

    /* Initialize B1 button GPIO */
    if (gpio_is_ready_dt(&button)) {
        gpio_pin_configure_dt(&button, GPIO_INPUT);
        gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
        gpio_add_callback(button.port, &button_cb);
        gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    }
    
    printk("[MODE] TEST\n");
    
    /* Start the 2-second timer and turn on blue LED for TEST mode */
    k_timer_start(&test_timer, K_SECONDS(2), K_SECONDS(2));
    if (rgb_ready) {
        set_color(0, 0, 1);
    }

    struct sensor_message msg;
    /* Deadline for the next statistics report */
    int64_t stats_deadline = 0;

    while (1) {
        /* Wait for a message or until the next statistics deadline in NORMAL mode */
        k_timeout_t timeout = K_FOREVER;
        if (current_mode == MODE_NORMAL) {
            int64_t remaining = stats_deadline - k_uptime_get();
            timeout = (remaining <= 0) ? K_NO_WAIT : K_MSEC(remaining);
        } 
        
        if (k_msgq_get(&sensor_msgq, &msg, timeout) == 0) {
            
            switch (msg.type) {
                case MSG_BUTTON_PRESSED:
                    /* Mode transition logic: TEST -> NORMAL -> SIMU -> TEST */
                    if (current_mode == MODE_TEST) {
                        current_mode = MODE_NORMAL;
                        k_timer_stop(&test_timer); /* Stop periodic print */
                        if (rgb_ready) {
                            set_color(0, 0, 0); /* Turn off TEST mode LED */
                        }
                        statistics_reset();
                        stats_deadline = k_uptime_get() + STATS_PERIOD_MS;
                    } else if (current_mode == MODE_NORMAL) {
                        current_mode = MODE_SIMU;
                    } else {
                        current_mode = MODE_TEST;
                        k_timer_start(&test_timer, K_SECONDS(2), K_SECONDS(2)); /* Restart periodic print */
                        if (rgb_ready) {
                            set_color(0, 0, 1); /* Turn on TEST mode LED */
                        }
                    }
                    
                    printk("[MODE] %s\n", 
                           current_mode == MODE_TEST ? "TEST" : 
                          (current_mode == MODE_NORMAL ? "NORMAL" : "SIMU"));
                    break;

                case MSG_TIMER_2S:
                    /* Periodic report output triggered by the timer */
                    if (current_mode == MODE_TEST) {
                        printk("\n--- [TEST MODE] REPORT ---\n");
                        printk("LDR: Brightness: %u%%\n", last_ldr.brightness);
                        printk("ENV: Temp: %.2f C | Hum: %.2f %%\n", 
                               (double)last_env.temperature, (double)last_env.humidity);
                        
                        printk("IMU: Gyro(deg/s) X=%.2f Y=%.2f Z=%.2f | Accel(m/s2) X=%.2f Y=%.2f Z=%.2f\n", 
                               (double)last_imu.measurements.gx, (double)last_imu.measurements.gy, (double)last_imu.measurements.gz,
                               (double)last_imu.measurements.ax, (double)last_imu.measurements.ay, (double)last_imu.measurements.az);
                        
                        if (last_gps.fix_valid) {
                            printk("GPS: Fix OK | UTC: %s | Sats: %u | Lat: %.6f | Lon: %.6f | Alt: %.1f m\n", 
                                   last_gps.utc, last_gps.satellites, last_gps.latitude, last_gps.longitude, (double)last_gps.altitude);
                        } else {
                            printk("GPS: Waiting for 3D fix...\n");
                        }
                        printk("-------------------------------\n");
                    }
                    break;

                case MSG_SENSOR_LDR:
                    last_ldr = msg.data.ldr;
                    
                    if (current_mode == MODE_NORMAL) {
                        statistics_add_light(msg.data.ldr.brightness);
                    }       
                    break;

                case MSG_SENSOR_MPU:
                    last_imu = msg.data.imu;
                    
                    if (current_mode == MODE_NORMAL) {
                        statistics_add_imu(
                        msg.data.imu.measurements.ax,
                        msg.data.imu.measurements.ay,
                        msg.data.imu.measurements.az,
                        msg.data.imu.measurements.gx,
                        msg.data.imu.measurements.gy,
                        msg.data.imu.measurements.gz
                        );
                    }       
                    break;

                case MSG_SENSOR_GPS:
                    last_gps = msg.data.gps;
                    break;

                case MSG_SENSOR_ENV:
                    last_env = msg.data.env;
                    
                    if (current_mode == MODE_NORMAL) {
                        statistics_add_environment(
                        msg.data.env.temperature,
                        msg.data.env.humidity
                        );
                    }       
                    break;
                    
                default:
                    printk("[MSGQ] Unknown message type received\n");
                    break;
            }
        }
        
        /* Print and reset statistics every 30 seconds in NORMAL mode */
        if (current_mode == MODE_NORMAL && k_uptime_get() >= stats_deadline) {
            statistics_print();
            statistics_reset();
            stats_deadline = k_uptime_get() + STATS_PERIOD_MS;
        }   
    }
    return 0;
}