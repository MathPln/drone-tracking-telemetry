#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>

struct gps_data {
    double latitude;
    double longitude;
    float altitude;
    uint8_t satellites;
    char utc[16];
    bool fix_valid;
};

#endif /* GPS_SENSOR_H */