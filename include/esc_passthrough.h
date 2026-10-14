// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "esc_backend.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct esc_passthrough esc_passthrough_t;

typedef enum {
    ESC_SESSION_NORMAL_EXIT,
    ESC_SESSION_TRANSPORT_LOSS,
    ESC_SESSION_TIMEOUT,
    ESC_SESSION_SETUP_FAILURE,
    ESC_SESSION_REPLY_FAILURE,
    ESC_SESSION_SHUTDOWN,
} esc_session_end_reason_t;

typedef struct {
    esc_session_end_reason_t reason;
    uint8_t bootloader_attempted_mask;
    uint8_t reboot_sent_mask;
    uint8_t needs_reboot_mask;
    bool io_released;
} esc_session_end_t;

typedef struct {
    void *ctx;
    esp_err_t (*acquire)(void *ctx);
    esp_err_t (*release)(void *ctx, const esc_session_end_t *result);
} esc_passthrough_lifecycle_t;

// stop must be idempotent and release all IO, even after a failed start.
// All wire callbacks except received_size are required. wire.release only
// releases the selected connection. ctx must outlive the session.
typedef struct {
    void *ctx;
    esp_err_t (*start)(void *ctx);
    esp_err_t (*stop)(void *ctx);
    esc_wire_t wire;
} esc_passthrough_io_t;

typedef struct {
    int pins[PT_MAX_CHANNELS]; // -1 disables a slot; enabled slots must be contiguous.
    int uart_port;
    uint32_t session_timeout_ms; // 0 disables the timeout.
    uint8_t uid[12];
    char board[4];
    pt_output_fn output;
    void *output_ctx;
    esc_passthrough_lifecycle_t lifecycle;
    const esc_passthrough_io_t *io; // NULL selects the built-in UART.
    pt_msp_command_fn application_command;
    void *application_ctx;
} esc_passthrough_config_t;

esc_passthrough_config_t esc_passthrough_config_default(void);
// Does not acquire IO or invoke lifecycle callbacks.
esp_err_t esc_passthrough_create(const esc_passthrough_config_t *config, esc_passthrough_t **out);
// Calls and motor setters must run on one task. Callbacks must not re-enter.
void esc_passthrough_feed(esc_passthrough_t *session, const uint8_t *data, size_t size, uint32_t now_ms);
void esc_passthrough_poll(esc_passthrough_t *session, uint32_t now_ms);
esp_err_t esc_passthrough_transport_lost(esc_passthrough_t *session);
esp_err_t esc_passthrough_shutdown(esc_passthrough_t *session);
// On failure, the object remains valid for cleanup retry.
esp_err_t esc_passthrough_destroy(esc_passthrough_t *session);
// Retries stop without repeating the lifecycle release callback.
esp_err_t esc_passthrough_retry_cleanup(esc_passthrough_t *session);
bool esc_passthrough_is_held(const esc_passthrough_t *session);
esp_err_t esc_passthrough_last_error(const esc_passthrough_t *session);
// Returns false until a session ends; the result survives reconnects.
bool esc_passthrough_last_end(const esc_passthrough_t *session, esc_session_end_t *result);
// Returns false until the channel has an InitFlash attempt; survives reconnects.
bool esc_passthrough_init_diagnostic(const esc_passthrough_t *session, uint8_t channel,
                                    esc_init_diagnostic_t *result);

#ifdef __cplusplus
}
#endif
