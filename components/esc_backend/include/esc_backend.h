// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "passthrough.h"

typedef struct {
    void *ctx;
    bool (*select)(void *ctx, uint8_t channel);
    bool (*send)(void *ctx, const uint8_t *data, size_t size);
    bool (*receive)(void *ctx, uint8_t *data, size_t size, unsigned timeout_ms);
    void (*release)(void *ctx);
    void (*pulse_low)(void *ctx, unsigned duration_ms);
} esc_wire_t;

typedef struct {
    esc_wire_t wire;
    bool connected;
    uint8_t mode;
} esc_backend_t;

void esc_backend_init(esc_backend_t *backend, const esc_wire_t *wire);
pt_esc_ops_t esc_backend_ops(esc_backend_t *backend);
