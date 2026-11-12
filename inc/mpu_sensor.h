#ifndef MPU_SENSOR_H
#define MPU_SENSOR_H

#include <stdint.h>

struct mpu6050_data {
    float ax, ay, az;  // Acceleration in m/s²
    float gx, gy, gz;  // Angular velocity in deg/s
};

int mpu6050_init(void);
int mpu6050_read(struct mpu6050_data *data);

#endif