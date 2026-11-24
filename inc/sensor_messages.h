#ifndef SENSOR_MESSAGES_H
#define SENSOR_MESSAGES_H

#include <zephyr/kernel.h>
#include <stdbool.h>
#include "gps_sensor.h"
#include "ldr_sensor.h"
#include "temp_hum_sensor.h"
#include "mpu_sensor.h"

enum message_type {
    MSG_SENSOR_LDR,
    MSG_SENSOR_MPU,
    MSG_SENSOR_GPS,
    MSG_SENSOR_ENV,
    MSG_BUTTON_PRESSED,
    MSG_TIMER_REPORT,
    MSG_SENSOR_FAULT
};

enum tracker_mode {
    MODE_TEST,
    MODE_NORMAL,
    MODE_SIMU
};

enum sensor_id {
    SENSOR_LDR,
    SENSOR_MPU,
    SENSOR_GPS,
    SENSOR_ENV
};

struct sensor_fault {
    enum sensor_id sensor;
    bool active;
    int error_code;
};

struct sensor_message {
    enum message_type type;
    union {
        struct ldr_data ldr;
        struct imu_message imu;
        struct gps_data gps;
        struct env_data env;
        struct sensor_fault fault;
        enum tracker_mode timer_mode;
    } data;
};

extern struct k_msgq sensor_msgq;

#endif /* SENSOR_MESSAGES_H */