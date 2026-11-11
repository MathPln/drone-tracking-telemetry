#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <zephyr/kernel.h>
#include <stdbool.h>

struct gps_data {
    double latitude;
    double longitude;
    float altitude;
    bool fix_valid;
};

#endif /* GPS_SENSOR_H */