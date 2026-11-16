
#ifndef SIMU_PROTOCOL_H
#define SIMU_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#include "gps_sensor.h"
#include "ldr_sensor.h"
#include "mpu_sensor.h"
#include "temp_hum_sensor.h"

/* SIMU frame dimensions */
#define SIMU_FRAME_LENGTH       221U
#define SIMU_FRAME_BUFFER_SIZE  (SIMU_FRAME_LENGTH + 1U)

/* Data required to generate one SIMU frame */
struct simu_data {
    uint16_t sequence;
    uint64_t timestamp_ms;

    struct gps_data gps;
    struct imu_message imu;
    struct ldr_data ldr;
    struct env_data env;

    /* Local position and velocity */
    float x, y, z;
    float vx, vy, vz;

    /* Tracker state: IDLE____, FLYING__, or PROBLEM_ */
    const char *state;

    /* 16-bit status flags */
    uint16_t flags;
};

/* Build one complete ASCII SIMU frame */
int simu_build_frame(const struct simu_data *data,
                     char *buffer,
                     size_t buffer_size);

#endif /* SIMU_PROTOCOL_H */
