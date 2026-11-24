#include "telemetry_report.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "simu_protocol.h"

int telemetry_report(enum tracker_mode mode,
                     const struct telemetry_data *data)
{
    if (data == NULL || data->ldr == NULL ||
        data->imu == NULL || data->gps == NULL ||
        data->env == NULL || data->tracker_state == NULL) {
        return -EINVAL;
    }

    if (mode == MODE_TEST) {
        printk("\n--- [TEST MODE] REPORT ---\n");
        printk("LDR: Brightness: %u%%\n", data->ldr->brightness);
        printk("ENV: Temp: %.2f C | Hum: %.2f %%\n",
               (double)data->env->temperature,
               (double)data->env->humidity);

        printk("IMU: Gyro(deg/s) X=%.2f Y=%.2f Z=%.2f | Accel(m/s2) X=%.2f Y=%.2f Z=%.2f\n",
               (double)data->imu->measurements.gx,
               (double)data->imu->measurements.gy,
               (double)data->imu->measurements.gz,
               (double)data->imu->measurements.ax,
               (double)data->imu->measurements.ay,
               (double)data->imu->measurements.az);

        if (data->gps->fix_valid) {
            printk("GPS: Fix OK | UTC: %s | Sats: %u | Lat: %.6f | Lon: %.6f | Alt: %.1f m\n",
                   data->gps->utc, data->gps->satellites,
                   data->gps->latitude, data->gps->longitude,
                   (double)data->gps->altitude);
        } else {
            printk("GPS: Waiting for 3D fix...\n");
        }
        printk("-------------------------------\n");
        return 0;
    }

    if (mode == MODE_NORMAL) {
        printk("\n--- [NORMAL MODE] REPORT ---\n");
        printk("LDR: Brightness: %u%%\n", data->ldr->brightness);
        printk("ENV: Temp: %.2f C | Hum: %.2f %%\n",
               (double)data->env->temperature,
               (double)data->env->humidity);

        printk("IMU: Gyro(deg/s) X=%.2f Y=%.2f Z=%.2f | Accel(m/s2) X=%.2f Y=%.2f Z=%.2f\n",
               (double)data->imu->measurements.gx,
               (double)data->imu->measurements.gy,
               (double)data->imu->measurements.gz,
               (double)data->imu->measurements.ax,
               (double)data->imu->measurements.ay,
               (double)data->imu->measurements.az);

        if (data->gps->fix_valid) {
            printk("GPS: Fix OK | UTC: %s | Sats: %u | Lat: %.6f | Lon: %.6f | Alt: %.1f m\n",
                   data->gps->utc, data->gps->satellites,
                   data->gps->latitude, data->gps->longitude,
                   (double)data->gps->altitude);
        } else {
            printk("GPS: Waiting for 3D fix...\n");
        }

        printk("Tracker state: %s\n", data->tracker_state);
        printk("-------------------------------\n");
        return 0;
    }

    if (mode == MODE_SIMU) {
        static uint16_t sequence;
        struct simu_data frame_data = {
            .sequence = sequence++,
            .timestamp_ms = (uint64_t)k_uptime_get(),
            .gps = *data->gps,
            .imu = *data->imu,
            .ldr = *data->ldr,
            .env = *data->env,
            .state = data->tracker_state,
            .flags = data->gps->fix_valid ? 0x0000 : 0x0002
        };
        static char frame[SIMU_FRAME_BUFFER_SIZE];
        int err = simu_build_frame(&frame_data, frame, sizeof(frame));
        if (err < 0) {
            return err;
        }
        printk("%s", frame);
        return 0;
    }

    return -EINVAL;
}
