#ifndef TEMP_HUM_SENSOR_H
#define TEMP_HUM_SENSOR_H

#include <zephyr/kernel.h>

struct env_data {
    float temperature;
    float humidity;
};

#endif /* TEMP_HUM_SENSOR_H */