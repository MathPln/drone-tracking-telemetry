#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "gps_sensor.h"

#define UART_NODE DT_ALIAS(gps_uart)
static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

#define NMEA_BUFFER_SIZE 128

/* Convert NMEA coordinates (DDMM.MMMM) to standard decimal degrees */
static double convert_nmea_to_decimal(double nmea_val, char direction)
{
    int degrees = (int)(nmea_val / 100.0);
    double minutes = nmea_val - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

void gps_thread_entry(void *p1, void *p2, void *p3)
{
    if (!device_is_ready(uart_dev)) {
        printk("[Error]: GPS UART device not ready\n");
        return;
    }

    struct gps_data current_gps = {0};
    char nmea_buffer[NMEA_BUFFER_SIZE];
    char last_gga_time[16] = {0};
    int buf_idx = 0;
    uint8_t rx_char;

    while (1) {
        /* Poll UART for incoming NMEA characters */
        if (uart_poll_in(uart_dev, &rx_char) == 0) {
            if (rx_char == '\n' || rx_char == '\r') {
                nmea_buffer[buf_idx] = '\0';

                // Debug
                // if (buf_idx > 0) {
                //     printk("[GPS RAW]: %s\n", nmea_buffer);
                // }

                /* Filter for GPGGA sentences (fix data) */
                if (strncmp(nmea_buffer, "$GPGGA", 6) == 0) {
                    char time_str[16], lat_dir, lon_dir, fix_quality_str[4];
                    float raw_lat = 0.0f, raw_lon = 0.0f;
                    float altitude = 0.0f;
                    int fix_quality = 0;

                    int parsed = sscanf(nmea_buffer,
                                        "$GPGGA,%15[^,],%f,%c,%f,%c,%3[^,],%*[^,],%*[^,],%f",
                                        time_str, &raw_lat, &lat_dir, &raw_lon, &lon_dir,
                                        fix_quality_str, &altitude);

                    if (parsed == 7) {
                        if (strcmp(time_str, last_gga_time) == 0) {
                            buf_idx = 0;
                            continue;
                        }
                        snprintf(last_gga_time, sizeof(last_gga_time), "%s", time_str);

                        fix_quality = atoi(fix_quality_str);

                        /* Check if satellites have a valid fix */
                        if (fix_quality > 0) {
                            current_gps.fix_valid = true;
                            current_gps.latitude = convert_nmea_to_decimal((double)raw_lat, lat_dir);
                            current_gps.longitude = convert_nmea_to_decimal((double)raw_lon, lon_dir);
                            current_gps.altitude = (double)altitude;
                            printk("[Info]: GPS Fix Valid | UTC: %s, Lat: %.6f, Lon: %.6f, Alt: %.1f m\n",
                                   time_str, current_gps.latitude, current_gps.longitude,
                                   (double)current_gps.altitude);
                        } else {
                            current_gps.fix_valid = false;
                            printk("[Info]: GPS waiting for 3D fix...\n");
                        }
                    }
                }
                buf_idx = 0; /* Reset line buffer */
            } else {
                if (buf_idx < (NMEA_BUFFER_SIZE - 1)) {
                    nmea_buffer[buf_idx++] = rx_char;
                } else {
                    buf_idx = 0; /* Buffer overflow guard */
                }
            }
        } else {
            k_sleep(K_TICKS(1)); /* Sleep CPU quickly if no data available */
        }
    }
}

K_THREAD_DEFINE(gps_thread_id, 2048, gps_thread_entry, NULL, NULL, NULL, 7, 0, 0);