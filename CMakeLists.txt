# SPDX-License-Identifier: GPL-3.0-or-later
cmake_minimum_required(VERSION 3.22)

if(NOT IDF_TARGET STREQUAL "esp32c3" AND NOT IDF_TARGET STREQUAL "esp32s3")
    message(FATAL_ERROR "ESC passthrough supports ESP32-C3 and ESP32-S3")
endif()

idf_component_register(
    SRCS "src/passthrough.c" "src/esc_backend.c"
         "src/esc_passthrough.c" "src/esc_uart.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_timer esp_hal_uart freertos
)
