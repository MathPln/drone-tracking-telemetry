#ifndef TELEMETRY_REPORT_H
#define TELEMETRY_REPORT_H

#include <stdint.h>
#include "sensor_messages.h"
#include "motion_estimator.h"

struct telemetry_data {
    const struct ldr_data *ldr;
    const struct imu_message *imu;
    const struct gps_data *gps;
    const struct env_data *env;
    const struct motion_estimate *motion;
    const char *tracker_state;
    uint16_t simu_flags;
};

int telemetry_report(enum tracker_mode mode,
                     const struct telemetry_data *data);

#endif /* TELEMETRY_REPORT_H */