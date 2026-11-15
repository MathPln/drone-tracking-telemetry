#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <stdint.h>

#include "statistics.h"

/* Statistics structure for one measurement */

struct statistic {
    double sum;
    double min;
    double max;
    uint32_t count;
};

/* Statistics accumulators for each sensor measurement */
static struct statistic light_stats;
static struct statistic temperature_stats;
static struct statistic humidity_stats;
static struct statistic acceleration_stats;
static struct statistic angular_velocity_stats;

/* Reset a statistics accumulator */
static void reset_statistic(struct statistic *s)
{
    s->sum = 0.0;
    s->min = 0.0;
    s->max = 0.0;
    s->count = 0;
}

/* Update the sum, minimum, maximum and sample count */
static void update_statistic(struct statistic *s, double value)
{
    if (s->count == 0) {
        s->min = value;
        s->max = value;
    } else {
        if (value < s->min) s->min = value;
        if (value > s->max) s->max = value;
    }

    s->sum += value;
    s->count++;
}

/* Initialize the statistics module */
void statistics_init(void)
{
    statistics_reset();
}
/* Reset all sensor statistics */
void statistics_reset(void)
{
    reset_statistic(&light_stats);
    reset_statistic(&temperature_stats);
    reset_statistic(&humidity_stats);
    reset_statistic(&acceleration_stats);
    reset_statistic(&angular_velocity_stats);
}
/* Add a light brightness measurement */
void statistics_add_light(double brightness)
{
    update_statistic(&light_stats, brightness);
}
/* Add temperature and humidity measurements */
void statistics_add_environment(double temperature, double humidity)
{
    update_statistic(&temperature_stats, temperature);
    update_statistic(&humidity_stats, humidity);
}
/* Calculate and record acceleration and angular velocity magnitudes */
void statistics_add_imu(double ax, double ay, double az,
                        double gx, double gy, double gz)
{
    double acceleration = sqrt(ax * ax + ay * ay + az * az);
    double angular_velocity = sqrt(gx * gx + gy * gy + gz * gz);

    update_statistic(&acceleration_stats, acceleration);
    update_statistic(&angular_velocity_stats, angular_velocity);
}


/* Print the statistics of one measurement */
static void print_statistic(const char *name, const char *unit,
                            const struct statistic *s)
{
    if (s->count == 0) {
        printk("%s: No data\n", name);
        return;
    }

    printk("%s: Mean=%.2f %s | Min=%.2f | Max=%.2f\n",
           name, s->sum / s->count, unit, s->min, s->max);
}
/* Print the statistics report */
void statistics_print(void)
{
    printk("\n========== STATISTICS (Last 30 seconds) ==========\n");

    print_statistic("Light", "%", &light_stats);
    print_statistic("Temperature", "C", &temperature_stats);
    print_statistic("Humidity", "%", &humidity_stats);
    print_statistic("Acceleration norm", "m/s2", &acceleration_stats);
    print_statistic("Angular velocity norm", "deg/s",
                    &angular_velocity_stats);

    printk("==================================================\n");
}