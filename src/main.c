#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <math.h>
#include "sensor_messages.h"
#include "statistics.h"
#include "rgb_led.h"


/* Central message queue definition */
K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_message), 10, 4);

/* B1 button configuration */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb;

/* Operating modes selected with button B1 */
enum tracker_mode { MODE_TEST, MODE_NORMAL, MODE_SIMU };
static enum tracker_mode current_mode = MODE_TEST;

/* Tracker states managed by the central state machine */
enum tracker_state { STATE_IDLE, STATE_FLYING, STATE_PROBLEM };
static enum tracker_state current_state = STATE_IDLE;

/* Statistics collection period in NORMAL mode */
#define STATS_PERIOD_MS 30000

/* Minimum duration of a sensor fault before entering PROBLEM state */
#define FAULT_PERSISTENCE_MS 6000

/* Motion detection thresholds */
#define GYRO_MOVEMENT_THRESHOLD  6.0f
#define GYRO_IDLE_THRESHOLD     3.0f

#define ACCEL_REST_MAGNITUDE    10.4f
#define ACCEL_MOVEMENT_DELTA     1.0f
#define ACCEL_IDLE_DELTA        0.5f

/* Movement and stationary conditions must persist for these durations */
#define MOVEMENT_PERSISTENCE_MS  500
#define IDLE_PERSISTENCE_MS     2000

/* Motion detection state and timers */
static bool motion_active = false;
static int64_t movement_start_ms = 0;
static int64_t idle_start_ms = 0;



/* Track active faults and their starting times */
static bool sensor_fault_active[4] = {false};
static int64_t sensor_fault_start[4] = {0};


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


/* Detect movement using gyroscope and accelerometer */
static void update_motion_detection(const struct imu_message *imu)
{
    const struct mpu6050_data *m = &imu->measurements;

    /* Correct the gyroscope values using the measured resting offsets */
    float gx = m->gx + 1.40f;
    float gy = m->gy - 1.21f;
    float gz = m->gz + 0.69f;

    /* Gyroscope magnitude */
    float gyro_magnitude = sqrtf(gx * gx + gy * gy + gz * gz);

    /* Acceleration magnitude */
    float accel_magnitude = sqrtf(
        m->ax * m->ax +
        m->ay * m->ay +
        m->az * m->az
    );

    /* Difference from the measured resting acceleration */
    float accel_delta = fabsf(accel_magnitude - ACCEL_REST_MAGNITUDE);

    bool movement_detected =
        (gyro_magnitude > GYRO_MOVEMENT_THRESHOLD) ||
        (accel_delta > ACCEL_MOVEMENT_DELTA);

    bool stationary =
        (gyro_magnitude < GYRO_IDLE_THRESHOLD) &&
        (accel_delta < ACCEL_IDLE_DELTA);

    int64_t now = k_uptime_get();
    /* Enter the movement state only after continuous movement detection */
    if (!motion_active) {
        idle_start_ms = 0;

        if (movement_detected) {
            if (movement_start_ms == 0) {
                movement_start_ms = now;
            }

            if (now - movement_start_ms >= MOVEMENT_PERSISTENCE_MS) {
                motion_active = true;
                movement_start_ms = 0;
                printk("[MOTION] Movement detected\n");
            }
        } else {
            movement_start_ms = 0;
        }
    } else {
        movement_start_ms = 0;
        /* Return to stationary state only after continuous inactivity */
        if (stationary) {
            if (idle_start_ms == 0) {
                idle_start_ms = now;
            }

            if (now - idle_start_ms >= IDLE_PERSISTENCE_MS) {
                motion_active = false;
                idle_start_ms = 0;
                printk("[MOTION] Movement stopped\n");
            }
        } else {
            idle_start_ms = 0;
        }
    }
}





int main(void)
{
    printk("[INFO] Main thread started - MSGQ initialized\n");
    /* Initialize the statistics module */
    statistics_init();
    if (rgb_led_init() != 0) {
        printk("[RGB] Initialization failed\n");
        return -1;
    }
    
    printk("[RGB] Initialization successful\n");
    /* TEST mode is indicated by the blue LED */
    rgb_led_set(false, false, true);


    /* Initialize B1 button GPIO */
    if (gpio_is_ready_dt(&button)) {
        gpio_pin_configure_dt(&button, GPIO_INPUT);
        gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
        gpio_add_callback(button.port, &button_cb);
        gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    }
    
    printk("[MODE] TEST\n");

    struct sensor_message msg;
    * Deadline for the next statistics report in NORMAL mode */
    int64_t stats_deadline = 0;


    while (1) {
        /* Wait for a message or until the next statistics deadline in NORMAL mode */
        k_timeout_t timeout = K_MSEC(500);

        if (current_mode == MODE_NORMAL) {
            int64_t remaining = stats_deadline - k_uptime_get();

            if (remaining <= 0) {
                timeout = K_NO_WAIT;
            } else if (remaining < 500) {
                timeout = K_MSEC(remaining);
            } else {
                timeout = K_MSEC(500);
            }
        }


        if (k_msgq_get(&sensor_msgq, &msg, timeout) == 0) {
        
            
            switch (msg.type) {
                case MSG_BUTTON_PRESSED:
                    /* Mode transition logic: TEST -> NORMAL -> SIMU -> TEST */
                    if (current_mode == MODE_TEST) {
                        current_mode = MODE_NORMAL;

                        statistics_reset();
                        stats_deadline = k_uptime_get() + STATS_PERIOD_MS;
                    } else if (current_mode == MODE_NORMAL) {
                        current_mode = MODE_SIMU;
                    } else {
                        current_mode = MODE_TEST;
                    }

                    printk("[MODE] %s\n", current_mode == MODE_TEST ? "TEST" :
                        (current_mode == MODE_NORMAL ? "NORMAL" : "SIMU"));

                    /* Update LED according to the selected mode */
                    if (current_mode == MODE_TEST) {
                        rgb_led_set(false, false, true);
                    } else if (current_mode == MODE_NORMAL) {
                        switch (current_state) {
                            case STATE_IDLE:
                            rgb_led_set(false, true, false);
                            break;
                        case STATE_FLYING:
                            rgb_led_set(true, true, false);
                            break;
                        case STATE_PROBLEM:
                            rgb_led_set(true, false, false);
                            break;
                        }
                    }

                    break;

                case MSG_SENSOR_LDR:
                    printk("[MSGQ LDR] Raw: %u | Brightness: %u%%\n", 
                           msg.data.ldr.raw, 
                           msg.data.ldr.brightness);
                    if (current_mode == MODE_NORMAL) {
                        statistics_add_light(msg.data.ldr.brightness);
                    }       
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
                    update_motion_detection(&msg.data.imu);
            
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
                    if (current_mode == MODE_NORMAL) {
                        statistics_add_environment(
                        msg.data.env.temperature,
                        msg.data.env.humidity
                        );
                    }       
                    break;

                case MSG_SENSOR_FAULT:
                    if (msg.data.fault.sensor < 4) {
                        enum sensor_id id = msg.data.fault.sensor;

                        if (msg.data.fault.active) {  
                            if (!sensor_fault_active[id]) {
                                sensor_fault_start[id] = k_uptime_get();
                                printk("[FAULT] Sensor %d error: %d\n",
                                        id, msg.data.fault.error_code);
                            }

                            sensor_fault_active[id] = true;
                        } else {
                            sensor_fault_active[id] = false;
                            sensor_fault_start[id] = 0;

                            printk("[FAULT] Sensor %d recovered\n", id);
                        }
                    }
                    break;

                                    
                default:
                    printk("[MSGQ] Unknown message type received\n");
                    break;
            }


        }

        /* Check whether a sensor fault has persisted */
        bool persistent_fault = false;

        for (int i = 0; i < 4; i++) {
            if (sensor_fault_active[i] &&
            (k_uptime_get() - sensor_fault_start[i]) >= FAULT_PERSISTENCE_MS) {
            persistent_fault = true;
            break;
            }
        }

        /* Determine tracker state: PROBLEM has the highest priority */
        enum tracker_state next_state;

        if (persistent_fault) {
            next_state = STATE_PROBLEM;
        } else if (motion_active) {
            next_state = STATE_FLYING;
        } else {
            next_state = STATE_IDLE;
        }

        /* Update tracker state */
        if (next_state != current_state) {
            current_state = next_state;

            switch (current_state) {
                case STATE_IDLE:
                printk("[STATE] IDLE\n");
                break;

                case STATE_FLYING:
                printk("[STATE] FLYING\n");
                break;

                case STATE_PROBLEM:
                printk("[STATE] PROBLEM\n");
                break;
            }

            /* Update state LED only in NORMAL mode */
            if (current_mode == MODE_NORMAL) {
                switch (current_state) {
                    case STATE_IDLE:
                    rgb_led_set(false, true, false);
                    break;

                    case STATE_FLYING:
                    rgb_led_set(true, true, false);
                    break;

                    case STATE_PROBLEM:
                    rgb_led_set(true, false, false);
                    break;
                }
            }
        }

        /* Print and reset statistics every 30 seconds in NORMAL mode */

        if (current_mode == MODE_NORMAL &&
            k_uptime_get() >= stats_deadline) {

            statistics_print();
            statistics_reset();
            stats_deadline = k_uptime_get() + STATS_PERIOD_MS;
            }   
    }
    return 0;
}