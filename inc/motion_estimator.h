#ifndef MOTION_ESTIMATOR_H
#define MOTION_ESTIMATOR_H

#include <stdbool.h>
#include "mpu_sensor.h"

struct motion_estimate {
    float ax;
    float ay;
    float az;

    float vx;
    float vy;
    float vz;

    float x;
    float y;
    float z;
};

void motion_estimator_init(void);

void motion_estimator_update(const struct imu_message *imu,
                             float dt,
                             bool stationary);

void motion_estimator_get(struct motion_estimate *estimate);

#endif /* MOTION_ESTIMATOR_H */