#ifndef SENSOR_MESSAGES_H
#define SENSOR_MESSAGES_H

#include <zephyr/kernel.h>
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
    MSG_TIMER_2S
};

struct sensor_message {
    enum message_type type;
    union {
        struct ldr_data ldr;
        struct imu_message imu;
        struct gps_data gps;
        struct env_data env;
    } data;
};

extern struct k_msgq sensor_msgq;

#endif /* SENSOR_MESSAGES_H */