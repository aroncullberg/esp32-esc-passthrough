// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "driver/uart.h"
extern uart_dev_t fake_hw;
#define UART_LL_GET_HW(port) ((void)(port), &fake_hw)
size_t uart_ll_get_txfifo_len(uart_dev_t *hw);
void uart_ll_write_txfifo(uart_dev_t *hw, const uint8_t *data, size_t size);
bool uart_ll_is_tx_idle(uart_dev_t *hw);
void uart_ll_txfifo_rst(uart_dev_t *hw);
