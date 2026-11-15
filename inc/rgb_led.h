#ifndef RGB_LED_H
#define RGB_LED_H

#include <stdbool.h>

int rgb_led_init(void);
void rgb_led_set(bool red, bool green, bool blue);

#endif /* RGB_LED_H */