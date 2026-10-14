// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "esc_backend.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int pins[PT_MAX_CHANNELS];
    int count, selected, port;
    unsigned claimed_mask;
    int64_t boot_ready_us;
    size_t received;
    bool installed, active;
    esp_err_t io_error;
} esc_uart_t;
bool esc_uart_pin_allowed(void *ctx, int pin);
esp_err_t esc_uart_init(esc_uart_t *link, const int pins[PT_MAX_CHANNELS], int count, int port);
esp_err_t esc_uart_start(void *ctx);
esp_err_t esc_uart_stop(void *ctx);
esc_wire_t esc_uart_wire(esc_uart_t *link);

#ifdef __cplusplus
}
#endif
