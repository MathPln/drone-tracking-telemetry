# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(emproject)

target_sources(app PRIVATE src/main.c src/environment_sensors.c src/gps_sensor.c src/mpu_sensor.c src/rgb_led.c src/statistics.c src/simu_protocol.c src/motion_estimator.c)

target_include_directories(app PRIVATE inc)
