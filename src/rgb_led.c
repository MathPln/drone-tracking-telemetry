#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <errno.h>

#include "rgb_led.h"

/* RGB LED GPIO specifications from device-tree aliases */
static const struct gpio_dt_spec led_red =
    GPIO_DT_SPEC_GET(DT_ALIAS(led_red), gpios);

static const struct gpio_dt_spec led_green =
    GPIO_DT_SPEC_GET(DT_ALIAS(led_green), gpios);

static const struct gpio_dt_spec led_blue =
    GPIO_DT_SPEC_GET(DT_ALIAS(led_blue), gpios);


/* Initialize the RGB LED GPIO pins */
int rgb_led_init(void)
{
    /* Check GPIO device readiness */
    if (!gpio_is_ready_dt(&led_red) ||
        !gpio_is_ready_dt(&led_green) ||
        !gpio_is_ready_dt(&led_blue)) {
        return -ENODEV;
    }

    /* Configure RGB pins as inactive outputs */
    int err = gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        return err;
    }

    err = gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        return err;
    }

    err = gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        return err;
    }

    return 0;
}
/* Set the RGB LED color using red, green and blue components */
void rgb_led_set(bool red, bool green, bool blue)
{
    gpio_pin_set_dt(&led_red, red);
    gpio_pin_set_dt(&led_green, green);
    gpio_pin_set_dt(&led_blue, blue);
}