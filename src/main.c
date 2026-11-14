#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "sensor_messages.h"

/* Central message queue definition */
K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_message), 10, 4);

int main(void)
{
    printk("[INFO] Main thread started - MSGQ initialized\n");

    struct sensor_message msg;

    while (1) {
        /* Wait forever for any sensor to put data in the queue */
        if (k_msgq_get(&sensor_msgq, &msg, K_FOREVER) == 0) {
            
            switch (msg.type) {
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