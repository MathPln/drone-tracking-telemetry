
#include "simu_protocol.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

/* Length before the checksum marker '*' */
#define SIMU_PREFIX_LENGTH 216U

int simu_build_frame(const struct simu_data *data,
                     char *buffer,
                     size_t buffer_size)
{
    char state_field[9];
    size_t state_length;
    int prefix_length;
    int suffix_length;
    unsigned char checksum = 0;

    /* Check input parameters */
    if (data == NULL || buffer == NULL || data->state == NULL) {
        return -EINVAL;
    }

    if (buffer_size < SIMU_FRAME_BUFFER_SIZE) {
        return -ENOSPC;
    }

    /* Convert tracker state to an 8-character field */
    if (strcmp(data->state, "IDLE") == 0) {
        state_length = 4;
    } else if (strcmp(data->state, "FLYING") == 0) {
        state_length = 6;
    } else if (strcmp(data->state, "PROBLEM") == 0) {
        state_length = 7;
    } else {
        return -EINVAL;
    }

    memset(state_field, '_', 8);
    memcpy(state_field, data->state, state_length);
    state_field[8] = '\0';

    /* Build the frame payload */
    prefix_length = snprintf(
        buffer,
        buffer_size,
        "$SIMU1,"
        "%05u,%010llu,%1u,%02u,"
        "%+010.6f,%+011.6f,%+08.1f,"
        "%+08.2f,%+08.2f,%+08.2f,"
        "%+07.2f,%+07.2f,%+07.2f,"
        "%+07.2f,%+07.2f,%+07.2f,"
        "%+07.2f,%+07.2f,%+07.2f,"
        "%+07.1f,%+07.1f,%+07.1f,"
        "%05.1f,%+06.1f,%05.1f,"
        "%.8s,%04X",
        (unsigned int)data->sequence,
        (unsigned long long)data->timestamp_ms,
        data->gps.fix_valid ? 1U : 0U,
        data->gps.fix_valid ? (unsigned int)data->gps.satellites : 0U,
        data->gps.latitude,
        data->gps.longitude,
        data->gps.altitude,
        data->x, data->y, data->z,
        data->vx, data->vy, data->vz,
        data->imu.roll,
        data->imu.pitch,
        data->imu.yaw,
        data->imu.measurements.ax,
        data->imu.measurements.ay,
        data->imu.measurements.az,
        data->imu.measurements.gx,
        data->imu.measurements.gy,
        data->imu.measurements.gz,
        (double)data->ldr.brightness,
        data->env.temperature,
        data->env.humidity,
        state_field,
        (unsigned int)data->flags
    );

    /* Every field must have exactly its specified width */
    if (prefix_length != SIMU_PREFIX_LENGTH) {
        return -ERANGE;
    }

    /* XOR checksum: exclude '$', include all payload characters */
    for (size_t i = 1; i < SIMU_PREFIX_LENGTH; i++) {
        checksum ^= (unsigned char)buffer[i];
    }

    /* Append checksum and CRLF */
    suffix_length = snprintf(
        buffer + SIMU_PREFIX_LENGTH,
        buffer_size - SIMU_PREFIX_LENGTH,
        "*%02X\r\n",
        (unsigned int)checksum
    );

    if (suffix_length != 5) {
        return -ERANGE;
    }

    return 0;
}
