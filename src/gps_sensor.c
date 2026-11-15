#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>


#include "gps_sensor.h"
#include "sensor_messages.h"

#define UART_NODE DT_ALIAS(gps_uart)
static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

/* Send GPS fault status to the central message queue */
static void gps_report_fault(bool active, int error_code)
{
    struct sensor_message msg = {
        .type = MSG_SENSOR_FAULT,
        .data.fault = {
            .sensor = SENSOR_GPS,
            .active = active,
            .error_code = error_code
        }
    };

    k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT);
}


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
        gps_report_fault(true, -1);
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
                    float raw_lat = 0.0f, raw_lon = 0.0f, altitude = 0.0f;
                    int fix_quality = 0, satellites = 0;

                    int parsed = sscanf(nmea_buffer,
                                        "$GPGGA,%15[^,],%f,%c,%f,%c,%3[^,],%d,%*[^,],%f",
                                        time_str, &raw_lat, &lat_dir, &raw_lon, &lon_dir,
                                        fix_quality_str, &satellites, &altitude);

                    if (parsed >= 8) {
                        if (strcmp(time_str, last_gga_time) == 0) {
                            buf_idx = 0;
                            continue;
                        }
                        snprintf(last_gga_time, sizeof(last_gga_time), "%s", time_str);
                        snprintf(current_gps.utc, sizeof(current_gps.utc), "%s", time_str);
                        
                        fix_quality = atoi(fix_quality_str);
                        current_gps.satellites = (uint8_t)satellites;

                        /* Check if satellites have a valid fix */
                        if (fix_quality > 0) {
                            current_gps.fix_valid = true;
                            current_gps.latitude = convert_nmea_to_decimal((double)raw_lat, lat_dir);
                            current_gps.longitude = convert_nmea_to_decimal((double)raw_lon, lon_dir);
                            current_gps.altitude = altitude;
                        } else {
                            current_gps.fix_valid = false;
                        }

                        struct sensor_message msg = {
                            .type = MSG_SENSOR_GPS,
                            .data.gps = current_gps
                        };
                        k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT);
                    }
                }
                buf_idx = 0; 
            } else {
                if (buf_idx < (NMEA_BUFFER_SIZE - 1)) {
                    nmea_buffer[buf_idx++] = rx_char;
                } else {
                    buf_idx = 0; 
                }
            }
        } else {
            k_sleep(K_TICKS(1)); 
        }
    }
}

K_THREAD_DEFINE(gps_thread_id, 2048, gps_thread_entry, NULL, NULL, NULL, 7, 0, 0);