# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 2.8.12)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(enverse_ble_intgrated)


target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/bma_sensor.c)
target_sources(app PRIVATE src/bma4.c)
target_sources(app PRIVATE src/bma456.c)
target_sources(app PRIVATE src/max30102.c)
target_sources(app PRIVATE src/latchSensor.c)
target_sources(app PRIVATE src/max30208.c)
target_sources(app PRIVATE src/lightSensor.c)
target_sources(app PRIVATE src/algorithm.c)
target_sources(app PRIVATE src/display.c)
target_sources(app PRIVATE src/lcd.c)
target_sources(app PRIVATE src/lcd.h)
target_sources(app PRIVATE src/display.h)
target_sources(app PRIVATE src/algorithm.h)
target_sources(app PRIVATE src/digitalPins.c)
target_sources(app PRIVATE src/digitalPins.h)
target_sources(app PRIVATE src/lightSensor.h)
target_sources(app PRIVATE src/max30208.h)
target_sources(app PRIVATE src/latchSensor.h)
target_sources(app PRIVATE src/max30102.h)
target_sources(app PRIVATE src/bma_sensor.h)
target_sources(app PRIVATE src/bma4.h)
target_sources(app PRIVATE src/bma456.h)
target_sources(app PRIVATE src/bma4_defs.h)



