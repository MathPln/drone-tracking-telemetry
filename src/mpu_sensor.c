
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

#include "mpu_sensor.h"

/* Semaphore synchronizing the timer with the IMU thread */
K_SEM_DEFINE(mpu6050_sample_sem, 0, 1);

/* I2C controller defined in the device tree */
#define I2C_NODE DT_ALIAS(sensor_i2c)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

#define MPU6050_ADDR 0x68

/* MPU6050 register addresses */
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B

/* Conversion constants for selected measurement ranges */
#define ACCEL_SCALE  16384.0f
#define GYRO_SCALE   131.0f
#define GRAVITY      9.80665f

/* Timer handler: wakes up the IMU thread */
static void mpu6050_timer_handler(struct k_timer *timer_id)
{
    k_sem_give(&mpu6050_sample_sem);
}

K_TIMER_DEFINE(mpu6050_timer, mpu6050_timer_handler, NULL);

/* Initialize MPU6050 registers */
int mpu6050_init(void)
{
    if (!device_is_ready(i2c_dev)) {
        printk("[Error]: I2C bus not ready for MPU6050\n");
        return -1;
    }

    /* Wake up the MPU6050 and select the X gyro clock */
    if (i2c_reg_write_byte(i2c_dev, MPU6050_ADDR,
                           REG_PWR_MGMT_1, 0x01) != 0) {
        return -1;
    }

    k_sleep(K_MSEC(100));

    /* Enable digital low-pass filter */
    if (i2c_reg_write_byte(i2c_dev, MPU6050_ADDR,
                           REG_CONFIG, 0x03) != 0) {
        return -1;
    }

    /* Set internal sampling rate to 10 Hz */
    if (i2c_reg_write_byte(i2c_dev, MPU6050_ADDR,
                           REG_SMPLRT_DIV, 99) != 0) {
        return -1;
    }

    /* Accelerometer range: ±2g */
    if (i2c_reg_write_byte(i2c_dev, MPU6050_ADDR,
                           REG_ACCEL_CONFIG, 0x00) != 0) {
        return -1;
    }

    /* Gyroscope range: ±250 degrees/s */
    if (i2c_reg_write_byte(i2c_dev, MPU6050_ADDR,
                           REG_GYRO_CONFIG, 0x00) != 0) {
        return -1;
    }

    printk("[Info]: MPU6050 initialized successfully\n");

    return 0;
}

/* Read and convert acceleration and angular velocity */
int mpu6050_read(struct mpu6050_data *data)
{
    if (data == NULL) {
        return -1;
    }

    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t buffer[14];

    if (i2c_write_read(i2c_dev, MPU6050_ADDR,
                       &reg, 1, buffer, sizeof(buffer)) != 0) {
        return -1;
    }

    /* Convert received bytes into signed raw values */
    int16_t raw_ax = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    int16_t raw_ay = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    int16_t raw_az = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    /* Bytes 6 and 7 contain temperature data (not used here) */

    int16_t raw_gx = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
    int16_t raw_gy = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    int16_t raw_gz = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);

    /* Convert acceleration from raw values to m/s² */
    data->ax = ((float)raw_ax / ACCEL_SCALE) * GRAVITY;
    data->ay = ((float)raw_ay / ACCEL_SCALE) * GRAVITY;
    data->az = ((float)raw_az / ACCEL_SCALE) * GRAVITY;

    /* Convert gyroscope values to degrees per second */
    data->gx = (float)raw_gx / GYRO_SCALE;
    data->gy = (float)raw_gy / GYRO_SCALE;
    data->gz = (float)raw_gz / GYRO_SCALE;

    return 0;
}

/* Independent MPU6050 acquisition thread */
void mpu6050_thread_entry(void *p1, void *p2, void *p3)
{
    printk("[Debug]: MPU6050 thread started\n");

    if (mpu6050_init() != 0) {
        printk("[Error]: MPU6050 initialization failed\n");
        return;
    }

    /* Initialize attitude angles */
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    /* Sampling period: 100 ms = 0.1 seconds */
    const float dt = 0.1f;



    /* Sample every 100 ms */
    k_timer_start(&mpu6050_timer, K_NO_WAIT, K_MSEC(100));

    while (1) {
        k_sem_take(&mpu6050_sample_sem, K_FOREVER);

        struct mpu6050_data current_data = {0};

        if (mpu6050_read(&current_data) == 0) {

            /* Calculate attitude using Euler integration */
            roll += current_data.gx * dt;
            pitch += current_data.gy * dt;
            yaw += current_data.gz * dt;





            printk("[Info]: Accel: X=%.2f Y=%.2f Z=%.2f m/s² | "
                   "Gyro: X=%.2f Y=%.2f Z=%.2f deg/s\n",
                   (double)current_data.ax,
                   (double)current_data.ay,
                   (double)current_data.az,
                   (double)current_data.gx,
                   (double)current_data.gy,
                   (double)current_data.gz);
                
                   
            printk("[Info]: Attitude: Roll=%.2f Pitch=%.2f Yaw=%.2f deg\n",
                   (double)roll,
                   (double)pitch,
                   (double)yaw);    


        } else {
            printk("[Error]: Failed to read MPU6050\n");
        }
    }
}

K_THREAD_DEFINE(mpu6050_thread_id, 1024,
                mpu6050_thread_entry,
                NULL, NULL, NULL,
                7, 0, 0);

