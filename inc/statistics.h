#ifndef STATISTICS_H
#define STATISTICS_H

void statistics_init(void);
void statistics_reset(void);

void statistics_add_light(double brightness);

void statistics_add_environment(double temperature,
                                double humidity);

void statistics_add_imu(double ax, double ay, double az,
                        double gx, double gy, double gz);

void statistics_print(void);

#endif /* STATISTICS_H */