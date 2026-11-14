#ifndef LDR_SENSOR_H
#define LDR_SENSOR_H

#include <stdint.h>

struct ldr_data {
    uint16_t raw;
    uint8_t brightness;
};

#endif /* LDR_SENSOR_H */