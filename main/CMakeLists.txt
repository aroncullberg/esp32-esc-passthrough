# SPDX-License-Identifier: GPL-3.0-or-later
idf_component_register(SRCS "main.c" "esc_uart.c" INCLUDE_DIRS "."
    REQUIRES protocol esc_backend esp_driver_uart esp_driver_gpio
             esp_driver_usb_serial_jtag esp_timer esp_hw_support esp_hal_uart)
