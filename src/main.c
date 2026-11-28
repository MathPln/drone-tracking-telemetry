#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <math.h>

#include "rgb_led.h"
#include "sensor_messages.h"
#include "statistics.h"
#include "motion_estimator.h"
#include "telemetry_report.h"

/* Central message queue definition */
K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_message), 10, 4);

volatile bool simu_mode_active = false;

/* B1 button configuration */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb;
static bool rgb_ready;

/* Operating modes selected with button B1 */
static volatile enum tracker_mode report_timer_mode = MODE_TEST;
static enum tracker_mode current_mode = MODE_TEST;

/* Tracker states managed by the central state machine */
enum tracker_state { STATE_IDLE, STATE_FLYING, STATE_PROBLEM };
static enum tracker_state current_state = STATE_IDLE;

static struct motion_estimate last_motion_estimate;
static int64_t last_motion_update_ms = 0;

/* Statistics collection period in NORMAL mode */
#define STATS_PERIOD_MS         30000

/* Minimum duration of a sensor fault before entering PROBLEM state */
#define FAULT_PERSISTENCE_MS    6000

/* Motion detection thresholds */
#define GYRO_MOVEMENT_THRESHOLD 1.0f
#define GYRO_IDLE_THRESHOLD     0.5f

#define ACCEL_REST_MAGNITUDE    9.81f
#define ACCEL_MOVEMENT_DELTA    0.15f
#define ACCEL_IDLE_DELTA        0.10f

/* Movement and stationary conditions must persist for these durations */
#define MOVEMENT_PERSISTENCE_MS 500
#define IDLE_PERSISTENCE_MS     2000

/* Motion detection state and timers */
static bool motion_active = false;
static int64_t movement_start_ms = 0;
static int64_t idle_start_ms = 0;

/* Track active faults and their starting times */
static bool sensor_fault_active[4] = {false};
static int64_t sensor_fault_start[4] = {0};

/* Cached sensor data for periodic reports */
static struct ldr_data last_ldr;
static struct imu_message last_imu;
static struct gps_data last_gps;
static struct env_data last_env;

/* Button interrupt callback */
static void button_pressed(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    static int64_t last_press_ms;
    int64_t now = k_uptime_get();

    /* 200 ms debounce */
    if (now - last_press_ms < 200) {
        return;
    }
    last_press_ms = now;

    /* Non-blocking send of the button event to the central queue */
    struct sensor_message msg = { .type = MSG_BUTTON_PRESSED };
    k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT);
}

/* One timer drives the active mode's periodic output */
static void report_timer_handler(struct k_timer *timer_id)
{
    struct sensor_message msg = {
        .type = MSG_TIMER_REPORT,
        .data.timer_mode = report_timer_mode
    };
    k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT);
}

K_TIMER_DEFINE(report_timer, report_timer_handler, NULL);

static void report_timer_start(k_timeout_t period)
{
    report_timer_mode = current_mode;
    k_timer_start(&report_timer, period, period);
}

/* Detect movement using gyroscope and accelerometer */
static void update_motion_detection(const struct imu_message *imu)
{
    const struct mpu6050_data *m = &imu->measurements;

    /* Gyroscope values are already calibrated in mpu_sensor.c */
    float gx = m->gx;
    float gy = m->gy;
    float gz = m->gz;

    float gyro_magnitude = sqrtf(gx * gx + gy * gy + gz * gz);
    float accel_magnitude = sqrtf(m->ax * m->ax +
                                  m->ay * m->ay +
                                  m->az * m->az);

    float accel_delta = fabsf(accel_magnitude - ACCEL_REST_MAGNITUDE);

    bool movement_detected =
        (gyro_magnitude > GYRO_MOVEMENT_THRESHOLD) ||
        (accel_delta > ACCEL_MOVEMENT_DELTA);

    bool stationary =
        (gyro_magnitude < GYRO_IDLE_THRESHOLD) &&
        (accel_delta < ACCEL_IDLE_DELTA);

    int64_t now = k_uptime_get();

    if (!motion_active) {
        idle_start_ms = 0;

        if (movement_detected) {
            if (movement_start_ms == 0) {
                movement_start_ms = now;
            }

            if (now - movement_start_ms >= MOVEMENT_PERSISTENCE_MS) {
                motion_active = true;
                movement_start_ms = 0;
                if (current_mode != MODE_SIMU) {
                    printk("[MOTION] Movement detected\n");
                }
            }
        } else {
            movement_start_ms = 0;
        }
    } else {
        movement_start_ms = 0;

        if (stationary) {
            if (idle_start_ms == 0) {
                idle_start_ms = now;
            }

            if (now - idle_start_ms >= IDLE_PERSISTENCE_MS) {
                motion_active = false;
                idle_start_ms = 0;
                if (current_mode != MODE_SIMU) {
                    printk("[MOTION] Movement stopped\n");
                }
            }
        } else {
            idle_start_ms = 0;
        }
    }
}



static uint16_t build_simu_flags(void)
{
    uint16_t flags = 0x0000;

    if (sensor_fault_active[SENSOR_MPU]) {
        flags |= 0x0001; /* Bit 0: IMU error */
    }
    if (!last_gps.fix_valid || sensor_fault_active[SENSOR_GPS]) {
        flags |= 0x0002; /* Bit 1: GPS invalid / no fix */
    }
    if (sensor_fault_active[SENSOR_LDR]) {
        flags |= 0x0004; /* Bit 2: LDR invalid */
    }
    if (sensor_fault_active[SENSOR_ENV]) {
        flags |= 0x0008; /* Bit 3: Si7021 error */
    }
    if (!last_gps.fix_valid) {
        flags |= 0x0040; /* Bit 6: Dead reckoning without GPS */
    }

    return flags;
}

int main(void)
{
    printk("[INFO] Main thread started - MSGQ initialized\n");

    /* Initialize the statistics module, motion estimator, and LEDs */
    statistics_init();
    motion_estimator_init();

    int rgb_error = rgb_led_init();
    if (rgb_error < 0) {
        printk("[ERROR] RGB LED initialization failed: %d\n", rgb_error);
    } else {
        rgb_ready = true;
    }

    /* Initialize B1 button GPIO */
    if (gpio_is_ready_dt(&button)) {
        gpio_pin_configure_dt(&button, GPIO_INPUT);
        gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
        gpio_add_callback(button.port, &button_cb);
        gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    }

    printk("[MODE] TEST\n");

    /* Start the 2-second report timer and turn on blue LED for TEST mode */
    report_timer_start(K_SECONDS(2));
    if (rgb_ready) {
        rgb_led_set(0, 0, 1);
    }

    struct sensor_message msg;
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
                        report_timer_start(K_SECONDS(5));
                        if (rgb_ready) {
                            if (current_state == STATE_IDLE) {
                                rgb_led_set(0, 1, 0);
                            } else if (current_state == STATE_FLYING) {
                                rgb_led_set(1, 1, 0);
                            } else {
                                rgb_led_set(1, 0, 0);
                            }
                        }
                        statistics_reset();
                        stats_deadline = k_uptime_get() + STATS_PERIOD_MS;
                    } else if (current_mode == MODE_NORMAL) {
                        current_mode = MODE_SIMU;
                        simu_mode_active = true;
                        if (rgb_ready) {
                            rgb_led_set(1, 1, 1);
                        }
                        /* Start SIMU timer at 10 Hz */
                        report_timer_start(K_MSEC(100));
                    } else {
                        current_mode = MODE_TEST;
                        simu_mode_active = false;
                        report_timer_start(K_SECONDS(2));
                        if (rgb_ready) {
                            rgb_led_set(0, 0, 1); /* Turn on TEST mode LED */
                        }
                    }

                    if (current_mode != MODE_SIMU) {
                        printk("[MODE] %s\n",
                               current_mode == MODE_TEST ? "TEST" : "NORMAL");
                    }
                    break;

                case MSG_TIMER_REPORT: {
                    if (msg.data.timer_mode != current_mode) {
                        break;
                    }

                    struct telemetry_data data = {
                        .ldr = &last_ldr,
                        .imu = &last_imu,
                        .gps = &last_gps,
                        .env = &last_env,
                        .motion = &last_motion_estimate,
                        .tracker_state =
                            current_state == STATE_IDLE ? "IDLE" :
                            current_state == STATE_FLYING ? "FLYING" : "PROBLEM",
                        .simu_flags = build_simu_flags()
                    };

                    int report_error = telemetry_report(current_mode, &data);
                    if (report_error < 0 && current_mode != MODE_SIMU) {
                        printk("[ERROR] Telemetry report failed: %d\n", report_error);
                    }
                    break;
                }

                case MSG_SENSOR_LDR:
                    last_ldr = msg.data.ldr;

                    if (current_mode == MODE_NORMAL) {
                        statistics_add_light(msg.data.ldr.brightness);
                    }
                    break;

                case MSG_SENSOR_MPU: {
                    last_imu = msg.data.imu;

                    /* Calculate actual elapsed time between IMU samples */
                    int64_t now = k_uptime_get();
                    float dt = (last_motion_update_ms == 0)
                        ? 0.05f
                        : (float)(now - last_motion_update_ms) / 1000.0f;

                    last_motion_update_ms = now;

                    /* Determine whether the current sample is stationary */
                    const struct mpu6050_data *m = &msg.data.imu.measurements;

                    float gyro_magnitude = sqrtf(m->gx * m->gx +
                                                 m->gy * m->gy +
                                                 m->gz * m->gz);

                    float accel_magnitude = sqrtf(m->ax * m->ax +
                                                  m->ay * m->ay +
                                                  m->az * m->az);

                    bool stationary = (gyro_magnitude < GYRO_IDLE_THRESHOLD) &&
                        (fabsf(accel_magnitude - ACCEL_REST_MAGNITUDE) < ACCEL_IDLE_DELTA);

                    /* Update position and velocity estimator */
                    motion_estimator_update(&msg.data.imu, dt, stationary);
                    motion_estimator_get(&last_motion_estimate);

                    update_motion_detection(&msg.data.imu);

                    if (current_mode == MODE_NORMAL) {
                        statistics_add_imu(
                            m->ax, m->ay, m->az,
                            m->gx, m->gy, m->gz
                        );
                    }
                    break;
                }

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

                case MSG_SENSOR_FAULT:
                    if (msg.data.fault.sensor < 4) {
                        enum sensor_id id = msg.data.fault.sensor;
                        if (msg.data.fault.active) {
                            if (!sensor_fault_active[id]) {
                                sensor_fault_start[id] = k_uptime_get();
                                if (current_mode != MODE_SIMU) {
                                    printk("[FAULT] Sensor %d error: %d\n", id, msg.data.fault.error_code);
                                }
                            }
                            sensor_fault_active[id] = true;
                        } else {
                            sensor_fault_active[id] = false;
                            sensor_fault_start[id] = 0;
                            if (current_mode != MODE_SIMU) {
                                printk("[FAULT] Sensor %d recovered\n", id);
                            }
                        }
                    }
                    break;

                default:
                    if (current_mode != MODE_SIMU) {
                        printk("[MSGQ] Unknown message type received\n");
                    }
                    break;
            }
        }

        /* Check whether a sensor fault has persisted */
        bool persistent_fault = false;
        for (int i = 0; i < 4; i++) {
            if (sensor_fault_active[i] && (k_uptime_get() - sensor_fault_start[i]) >= FAULT_PERSISTENCE_MS) {
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

            if (current_mode == MODE_NORMAL && rgb_ready) {
                switch (current_state) {
                    case STATE_IDLE:
                        rgb_led_set(0, 1, 0); /* Green */
                        break;

                    case STATE_FLYING:
                        rgb_led_set(1, 1, 0); /* Yellow */
                        break;

                    case STATE_PROBLEM:
                        rgb_led_set(1, 0, 0); /* Red */
                        break;
                }
            }

            if (current_mode != MODE_SIMU) {
                printk("[STATE] %s\n",
                       current_state == STATE_IDLE ? "IDLE" :
                       current_state == STATE_FLYING ? "FLYING" : "PROBLEM");
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
