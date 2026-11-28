#include <math.h>
#include <string.h>

#include "motion_estimator.h"

#define GRAVITY        9.80665f
#define DEG_TO_RAD     0.01745329252f
#define MAX_DT         0.2f
#define ACCEL_DEADBAND 0.25f


static struct motion_estimate estimate;

void motion_estimator_init(void)
{
    memset(&estimate, 0, sizeof(estimate));
}

void motion_estimator_update(const struct imu_message *imu,
                             float dt,
                             bool stationary)
{
    if (imu == NULL || dt <= 0.0f || dt > MAX_DT) {
        return;
    }

    float roll  = imu->roll  * DEG_TO_RAD;
    float pitch = imu->pitch * DEG_TO_RAD;
    float yaw   = imu->yaw   * DEG_TO_RAD;

    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    float ax = imu->measurements.ax;
    float ay = imu->measurements.ay;
    float az = imu->measurements.az;

    /* Rotation du repère capteur vers le repère terrestre (ZYX) */
    float world_ax =
        (cy * cp) * ax +
        (cy * sp * sr - sy * cr) * ay +
        (cy * sp * cr + sy * sr) * az;

    float world_ay =
        (sy * cp) * ax +
        (sy * sp * sr + cy * cr) * ay +
        (sy * sp * cr - cy * sr) * az;

    float world_az =
        (-sp) * ax +
        (cp * sr) * ay +
        (cp * cr) * az;

    /* Suppression de la gravité terrestre */
    float lin_ax = world_ax;
    float lin_ay = world_ay;
    float lin_az = world_az - GRAVITY;

    /* Zone morte pour éliminer le bruit du capteur */
    if (fabsf(lin_ax) < ACCEL_DEADBAND) lin_ax = 0.0f;
    if (fabsf(lin_ay) < ACCEL_DEADBAND) lin_ay = 0.0f;
    if (fabsf(lin_az) < ACCEL_DEADBAND) lin_az = 0.0f;

    estimate.ax = lin_ax;
    estimate.ay = lin_ay;
    estimate.az = lin_az;

    if (stationary && lin_ax == 0.0f && lin_ay == 0.0f && lin_az == 0.0f) {
        /* Arrêt complet confirmé : remise à zéro de la vitesse (ZUPT) */
        estimate.vx = 0.0f;
        estimate.vy = 0.0f;
        estimate.vz = 0.0f;
        return;
    }

    /* Intégration trapézoïdale : accélération -> vitesse */
    float prev_vx = estimate.vx;
    float prev_vy = estimate.vy;
    float prev_vz = estimate.vz;

    estimate.vx += estimate.ax * dt;
    estimate.vy += estimate.ay * dt;
    estimate.vz += estimate.az * dt;

    /* Léger amortissement si accélération nulle pour éviter la dérive infinie */
    if (estimate.ax == 0.0f) estimate.vx *= 0.90f;
    if (estimate.ay == 0.0f) estimate.vy *= 0.90f;
    if (estimate.az == 0.0f) estimate.vz *= 0.90f;

    /* Intégration : vitesse -> position */
    estimate.x += 0.5f * (prev_vx + estimate.vx) * dt;
    estimate.y += 0.5f * (prev_vy + estimate.vy) * dt;
    estimate.z += 0.5f * (prev_vz + estimate.vz) * dt;
}

void motion_estimator_get(struct motion_estimate *output)
{
    if (output != NULL) {
        *output = estimate;
    }
}
