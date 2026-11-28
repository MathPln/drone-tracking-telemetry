#include "telemetry_report.h"

#include <errno.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "simu_protocol.h"

static float clampf(float val, float min_val, float max_val)
{
    if (isnan(val) || isinf(val)) {
        return 0.0f;
    }
    if (val < min_val) {
        return min_val;
    }
    if (val > max_val) {
        return max_val;
    }
    return val;
}

static float wrap_angle_deg(float angle)
{
    if (isnan(angle) || isinf(angle)) {
        return 0.0f;
    }
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

int telemetry_report(enum tracker_mode mode,
                     const struct telemetry_data *data)
{
    if (data == NULL || data->ldr == NULL ||
        data->imu == NULL || data->gps == NULL ||
        data->env == NULL || data->motion == NULL ||
        data->tracker_state == NULL) {
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

        printk("Attitude(deg): Roll=%.2f | Pitch=%.2f | Yaw=%.2f\n",
               (double)data->imu->roll,
               (double)data->imu->pitch,
               (double)data->imu->yaw);

        printk("World Accel(m/s2): X=%.2f Y=%.2f Z=%.2f\n",
               (double)data->motion->ax,
               (double)data->motion->ay,
               (double)data->motion->az);

        printk("Velocity(m/s): X=%.3f Y=%.3f Z=%.3f\n",
               (double)data->motion->vx,
               (double)data->motion->vy,
               (double)data->motion->vz);

        printk("Position(m): X=%.3f Y=%.3f Z=%.3f\n",
               (double)data->motion->x,
               (double)data->motion->y,
               (double)data->motion->z);

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
        static char frame[SIMU_FRAME_BUFFER_SIZE];

        struct simu_data frame_data = {
            .sequence = sequence++,
            .timestamp_ms = (uint64_t)k_uptime_get(),
            .gps = *data->gps,
            .imu = *data->imu,
            .ldr = *data->ldr,
            .env = *data->env,
            .x = clampf(data->motion->x, -9999.99f, 9999.99f),
            .y = clampf(data->motion->y, -9999.99f, 9999.99f),
            .z = clampf(data->motion->z, -9999.99f, 9999.99f),
            .vx = clampf(data->motion->vx, -999.99f, 999.99f),
            .vy = clampf(data->motion->vy, -999.99f, 999.99f),
            .vz = clampf(data->motion->vz, -999.99f, 999.99f),
            .state = data->tracker_state,
            .flags = data->simu_flags
        };

        frame_data.imu.roll  = wrap_angle_deg(data->imu->roll);
        frame_data.imu.pitch = wrap_angle_deg(data->imu->pitch);
        frame_data.imu.yaw   = wrap_angle_deg(data->imu->yaw);

        int err = simu_build_frame(&frame_data, frame, sizeof(frame));
        if (err < 0) {
            return err;
        }
        printk("%s", frame);
        return 0;
    }

    return -EINVAL;
}