#ifndef MPU_SENSOR_H
#define MPU_SENSOR_H

#include <stdint.h>

struct mpu6050_data {
    float ax, ay, az;
    float gx, gy, gz;
};

struct imu_message {
    struct mpu6050_data measurements;
    float roll;
    float pitch;
    float yaw;
};

int mpu6050_init(void);
int mpu6050_read(struct mpu6050_data *data);

#endif /* MPU_SENSOR_H */