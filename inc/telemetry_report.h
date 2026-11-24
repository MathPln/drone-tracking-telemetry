#ifndef TELEMETRY_REPORT_H
#define TELEMETRY_REPORT_H

#include "sensor_messages.h"

struct telemetry_data {
    const struct ldr_data *ldr;
    const struct imu_message *imu;
    const struct gps_data *gps;
    const struct env_data *env;
    const char *tracker_state;
};

int telemetry_report(enum tracker_mode mode,
                     const struct telemetry_data *data);

#endif /* TELEMETRY_REPORT_H */
