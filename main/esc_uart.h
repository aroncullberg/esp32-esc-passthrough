// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "esc_backend.h"
#include "esp_err.h"

typedef struct { int pins[4]; int count; int selected; } esc_uart_t;
bool esc_uart_pin_allowed(void *ctx, int pin);
esp_err_t esc_uart_init(esc_uart_t *link, const int pins[4], int count);
esc_wire_t esc_uart_wire(esc_uart_t *link);
