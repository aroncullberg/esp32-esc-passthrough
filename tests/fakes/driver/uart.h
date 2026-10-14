// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
enum { UART_DATA_8_BITS, UART_PARITY_DISABLE, UART_STOP_BITS_1,
       UART_HW_FLOWCTRL_DISABLE, UART_SCLK_DEFAULT };
typedef struct { int baud_rate, data_bits, parity, stop_bits, flow_ctrl, source_clk; } uart_config_t;
typedef struct { int unused; } uart_dev_t;
bool uart_is_driver_installed(int port);
esp_err_t uart_driver_install(int port, int rx_size, int tx_size, int queue_size, void *queue, int flags);
esp_err_t uart_driver_delete(int port);
esp_err_t uart_param_config(int port, const uart_config_t *config);
esp_err_t uart_set_rx_full_threshold(int port, int threshold);
esp_err_t uart_set_rx_timeout(int port, int threshold);
esp_err_t uart_flush_input(int port);
int uart_read_bytes(int port, void *data, size_t size, uint32_t ticks);
