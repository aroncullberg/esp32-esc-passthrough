// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "passthrough.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void *ctx;
    bool (*select)(void *ctx, uint8_t channel);
    bool (*send)(void *ctx, const uint8_t *data, size_t size);
    bool (*receive)(void *ctx, uint8_t *data, size_t size, unsigned timeout_ms);
    void (*release)(void *ctx);
    bool (*pulse_low)(void *ctx, unsigned duration_ms);
    // Optional; includes bytes received before a timeout.
    size_t (*received_size)(void *ctx);
} esc_wire_t;

enum { ESC_INIT_ATTEMPTS = 3, ESC_INIT_REPLY_SIZE = 9, ESC_RX_SIZE_UNKNOWN = 255 };
typedef enum {
    ESC_INIT_NOT_ATTEMPTED, ESC_INIT_SELECT_FAILED, ESC_INIT_SEND_FAILED,
    ESC_INIT_RX_TIMEOUT, ESC_INIT_BAD_REPLY, ESC_INIT_BAD_SIGNATURE, ESC_INIT_OK,
} esc_init_result_t;
typedef struct {
    esc_init_result_t result;
    uint8_t received;
    uint8_t reply[ESC_INIT_REPLY_SIZE];
    uint8_t skipped;
    uint8_t prefix[ESC_INIT_REPLY_SIZE - 1];
} esc_init_attempt_t;
typedef struct {
    uint8_t attempts;
    esc_init_attempt_t attempt[ESC_INIT_ATTEMPTS];
} esc_init_diagnostic_t;

typedef struct {
    esc_wire_t wire;
    bool connected;
    uint8_t mode;
    esc_init_diagnostic_t init[PT_MAX_CHANNELS];
} esc_backend_t;

void esc_backend_init(esc_backend_t *backend, const esc_wire_t *wire);
pt_esc_ops_t esc_backend_ops(esc_backend_t *backend);

#ifdef __cplusplus
}
#endif
