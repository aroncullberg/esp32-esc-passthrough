// SPDX-License-Identifier: GPL-3.0-or-later
#include "esc_passthrough.h"
#include "esc_uart.h"
#include "sdkconfig.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

struct esc_passthrough {
    pt_t protocol;
    esc_backend_t backend;
    pt_esc_ops_t backend_ops;
    esc_uart_t uart;
    esc_passthrough_io_t io;
    esc_passthrough_lifecycle_t lifecycle;
    pt_output_fn output;
    void *output_ctx;
    pt_msp_command_fn application_command;
    void *application_ctx;
    uint32_t lease_ms, last_traffic_ms;
    uint8_t attempted, reboot_sent, needs_reboot;
    bool held, io_started, io_attempted, cleanup_pending, output_failed, has_end;
    esp_err_t last_error;
    esc_session_end_t last_end;
};

static void record_error(esc_passthrough_t *s, esp_err_t error)
{
    if (error != ESP_OK) s->last_error = error;
}

static esp_err_t activate_io(esc_passthrough_t *s)
{
    if (s->io_started) return ESP_OK;
    s->io_attempted = true;
    esp_err_t error = s->io.start(s->io.ctx);
    s->io_started = error == ESP_OK;
    record_error(s, error);
    return error;
}

static esp_err_t stop_io(esc_passthrough_t *s)
{
    if (!s->io_attempted && !s->cleanup_pending) return ESP_OK;
    esp_err_t error = s->io.stop(s->io.ctx);
    s->cleanup_pending = error != ESP_OK;
    s->io_started = false;
    if (error == ESP_OK) s->io_attempted = false;
    record_error(s, error);
    return error;
}

static esp_err_t end_session(esc_passthrough_t *s, esc_session_end_reason_t reason)
{
    if (!s->held) return s->cleanup_pending ? ESP_ERR_INVALID_STATE : ESP_OK;
    if (s->needs_reboot && !s->io_attempted) activate_io(s);
    if (s->io_started) {
        for (uint8_t ch = 0; ch < s->protocol.channels; ++ch) {
            uint8_t bit = 1u << ch;
            if (!(s->needs_reboot & bit)) continue;
            if (s->backend_ops.reset(s->backend_ops.ctx, ch, true)) {
                s->needs_reboot &= ~bit;
                s->reboot_sent |= bit;
            } else record_error(s, ESP_FAIL);
        }
        s->backend_ops.release(s->backend_ops.ctx);
    }
    esp_err_t cleanup = stop_io(s);
    // io_started gates release so it cannot touch the next owner's pins.
    pt_reset_session(&s->protocol);
    s->protocol.used = 0;
    s->held = false;
    s->last_end = (esc_session_end_t){.reason = reason,
        .bootloader_attempted_mask = s->attempted, .reboot_sent_mask = s->reboot_sent,
        .needs_reboot_mask = s->needs_reboot, .io_released = cleanup == ESP_OK};
    s->has_end = true;
    if (s->lifecycle.release)
        record_error(s, s->lifecycle.release(s->lifecycle.ctx, &s->last_end));
    return s->last_error;
}

static bool acquire(void *ctx)
{
    esc_passthrough_t *s = ctx;
    if (s->held) return true;
    if (s->cleanup_pending || s->output_failed) {
        record_error(s, ESP_ERR_INVALID_STATE); return false;
    }
    s->last_error = ESP_OK;
    esp_err_t error = s->lifecycle.acquire ? s->lifecycle.acquire(s->lifecycle.ctx) : ESP_OK;
    record_error(s, error);
    if (error != ESP_OK) return false;
    s->held = true;
    s->attempted = s->needs_reboot;
    s->reboot_sent = 0;
    return true;
}

static bool activate(void *ctx)
{
    return activate_io(ctx) == ESP_OK;
}

static void protocol_end(void *ctx, pt_session_end_reason_t reason)
{
    esc_session_end_reason_t mapped = reason == PT_SESSION_EXIT ? ESC_SESSION_NORMAL_EXIT :
        reason == PT_SESSION_SETUP_FAILURE ? ESC_SESSION_SETUP_FAILURE : ESC_SESSION_REPLY_FAILURE;
    end_session(ctx, mapped);
}

static void traffic(void *ctx, uint32_t now_ms)
{
    esc_passthrough_t *s = ctx;
    s->last_traffic_ms = now_ms;
}

static bool output(void *ctx, const uint8_t *data, size_t size)
{
    esc_passthrough_t *s = ctx;
    bool ok = s->output(s->output_ctx, data, size);
    if (!ok) { s->output_failed = true; record_error(s, ESP_FAIL); }
    return ok;
}

static bool connect_esc(void *ctx, uint8_t channel, uint8_t info[4])
{
    esc_passthrough_t *s = ctx;
    uint8_t bit = 1u << channel;
    s->attempted |= bit;
    s->needs_reboot |= bit; // The ESC may enter its bootloader even if the reply times out.
    return s->backend_ops.connect(s->backend_ops.ctx, channel, info);
}

static bool reset_esc(void *ctx, uint8_t channel, bool reboot)
{
    esc_passthrough_t *s = ctx;
    bool ok = s->backend_ops.reset(s->backend_ops.ctx, channel, reboot);
    if (ok && reboot) {
        s->needs_reboot &= ~(1u << channel);
        s->reboot_sent |= 1u << channel;
    }
    return ok;
}

static bool read_esc(void *ctx, uint16_t address, size_t size, uint8_t *data)
{
    esc_passthrough_t *s = ctx;
    return s->backend_ops.read(s->backend_ops.ctx, address, size, data);
}

static bool erase_esc(void *ctx, uint8_t page)
{
    esc_passthrough_t *s = ctx;
    return s->backend_ops.erase_page(s->backend_ops.ctx, page);
}

static bool write_esc(void *ctx, uint16_t address, size_t size, const uint8_t *data)
{
    esc_passthrough_t *s = ctx;
    return s->backend_ops.write(s->backend_ops.ctx, address, size, data);
}

static bool alive_esc(void *ctx)
{
    esc_passthrough_t *s = ctx;
    return s->backend_ops.alive(s->backend_ops.ctx);
}

static void release_wire(void *ctx)
{
    esc_passthrough_t *s = ctx;
    if (s->io_started) s->backend_ops.release(s->backend_ops.ctx);
}

static bool application_command(void *ctx, uint8_t command, const uint8_t *payload,
                                size_t size, uint8_t *reply, size_t capacity,
                                size_t *reply_size, bool *error)
{
    esc_passthrough_t *s = ctx;
    if (s->application_command && s->application_command(s->application_ctx,
        command, payload, size, reply, capacity, reply_size, error)) return true;
    if (command != 253) return false; // MSP_DEBUGMSG.
    *reply_size = 0;
    *error = false;
    if (size > 1 || (size && payload[0] >= s->protocol.channels)) {
        *error = true; return true;
    }
    uint8_t channel = size ? payload[0] : 0;
    const esc_init_diagnostic_t *d = &s->backend.init[channel];
    static const char *const names[] = {"none", "select-failed", "send-failed",
        "rx-timeout", "bad-reply", "bad-signature", "ok"};
    int written = snprintf((char *)reply, capacity, "ESC init ch=%u gpio=%d attempts=%u\n",
                           channel, s->uart.pins[channel], d->attempts);
    if (written < 0 || (size_t)written >= capacity) { *error = true; return true; }
    size_t used = written;
    for (unsigned i = 0; i < d->attempts; ++i) {
        const esc_init_attempt_t *a = &d->attempt[i];
        written = snprintf((char *)reply + used, capacity - used,
            "%u: %s rx=%u skip=%u noise=%02x bytes=", i + 1, names[a->result],
            a->received, a->skipped, a->skipped ? a->prefix[0] : 0);
        if (written < 0 || (size_t)written >= capacity - used) { *error = true; return true; }
        used += written;
        unsigned count = a->received == ESC_RX_SIZE_UNKNOWN ? 0 : a->received;
        for (unsigned j = 0; j < count; ++j) {
            written = snprintf((char *)reply + used, capacity - used, "%02x", a->reply[j]);
            if (written < 0 || (size_t)written >= capacity - used) { *error = true; return true; }
            used += written;
        }
        if (used + 1 >= capacity) { *error = true; return true; }
        reply[used++] = '\n';
    }
    *reply_size = used;
    return true;
}

esc_passthrough_config_t esc_passthrough_config_default(void)
{
    return (esc_passthrough_config_t){.pins = {-1, -1, -1, -1},
        .uart_port = CONFIG_ESC_PASSTHROUGH_UART,
        .session_timeout_ms = CONFIG_ESC_PASSTHROUGH_LEASE_MS,
        .board = {'E', 'S', 'P', '3'}};
}

esp_err_t esc_passthrough_create(const esc_passthrough_config_t *config, esc_passthrough_t **out)
{
    if (!config || !out) return ESP_ERR_INVALID_ARG;
    *out = NULL;
    int count = pt_validate_pins(config->pins, esc_uart_pin_allowed, NULL);
    if (count < 1 || !config->output ||
        (!!config->lifecycle.acquire != !!config->lifecycle.release)) return ESP_ERR_INVALID_ARG;
    if (config->io && (!config->io->start || !config->io->stop ||
        !config->io->wire.select || !config->io->wire.send || !config->io->wire.receive ||
        !config->io->wire.pulse_low || !config->io->wire.release)) return ESP_ERR_INVALID_ARG;
    esc_passthrough_t *s = calloc(1, sizeof(*s));
    if (!s) return ESP_ERR_NO_MEM;
    if (config->io) s->io = *config->io;
    else {
        esp_err_t error = esc_uart_init(&s->uart, config->pins, count, config->uart_port);
        if (error != ESP_OK) { free(s); return error; }
        s->io = (esc_passthrough_io_t){.ctx = &s->uart, .start = esc_uart_start,
            .stop = esc_uart_stop, .wire = esc_uart_wire(&s->uart)};
    }
    s->lifecycle = config->lifecycle;
    s->lease_ms = config->session_timeout_ms;
    s->output = config->output;
    s->output_ctx = config->output_ctx;
    s->application_command = config->application_command;
    s->application_ctx = config->application_ctx;
    memcpy(s->uart.pins, config->pins, sizeof(s->uart.pins));
    esc_backend_init(&s->backend, &s->io.wire);
    s->backend_ops = esc_backend_ops(&s->backend);
    pt_esc_ops_t ops = {.ctx = s, .connect = connect_esc, .read = read_esc,
        .erase_page = erase_esc, .write = write_esc, .alive = alive_esc,
        .reset = reset_esc, .release = release_wire};
    pt_init(&s->protocol, count, &ops, output, s, config->uid, config->board);
    pt_set_msp_handler(&s->protocol, application_command, s);
    pt_session_ops_t hooks = {.ctx = s, .acquire = acquire, .activate = activate,
        .end = protocol_end, .traffic = traffic};
    pt_set_session_ops(&s->protocol, &hooks);
    *out = s;
    return ESP_OK;
}

void esc_passthrough_feed(esc_passthrough_t *s, const uint8_t *data, size_t size, uint32_t now_ms)
{
    if (!s || !data) return;
    s->output_failed = false;
    pt_feed(&s->protocol, data, size, now_ms);
}

void esc_passthrough_poll(esc_passthrough_t *s, uint32_t now_ms)
{
    if (!s) return;
    pt_expire(&s->protocol, now_ms);
    if (s->held && s->lease_ms && (uint32_t)(now_ms - s->last_traffic_ms) >= s->lease_ms)
        end_session(s, ESC_SESSION_TIMEOUT);
}

esp_err_t esc_passthrough_transport_lost(esc_passthrough_t *s)
{
    return s ? end_session(s, ESC_SESSION_TRANSPORT_LOSS) : ESP_ERR_INVALID_ARG;
}

esp_err_t esc_passthrough_shutdown(esc_passthrough_t *s)
{
    return s ? end_session(s, ESC_SESSION_SHUTDOWN) : ESP_ERR_INVALID_ARG;
}

esp_err_t esc_passthrough_retry_cleanup(esc_passthrough_t *s)
{
    if (!s) return ESP_ERR_INVALID_ARG;
    if (s->held) return ESP_ERR_INVALID_STATE;
    esp_err_t error = stop_io(s);
    if (s->has_end) s->last_end.io_released = error == ESP_OK;
    return error;
}

esp_err_t esc_passthrough_destroy(esc_passthrough_t *s)
{
    if (!s) return ESP_ERR_INVALID_ARG;
    esp_err_t ended = esc_passthrough_shutdown(s);
    esp_err_t error = esc_passthrough_retry_cleanup(s);
    if (error != ESP_OK) return error;
    if (ended != ESP_OK) return ended;
    free(s);
    return ESP_OK;
}

bool esc_passthrough_is_held(const esc_passthrough_t *s)
{
    return s && s->held;
}

esp_err_t esc_passthrough_last_error(const esc_passthrough_t *s)
{
    return s ? s->last_error : ESP_ERR_INVALID_ARG;
}

bool esc_passthrough_last_end(const esc_passthrough_t *s, esc_session_end_t *result)
{
    if (!s || !result || !s->has_end) return false;
    *result = s->last_end;
    return true;
}

bool esc_passthrough_init_diagnostic(const esc_passthrough_t *s, uint8_t channel,
                                    esc_init_diagnostic_t *result)
{
    if (!s || !result || channel >= s->protocol.channels || !s->backend.init[channel].attempts)
        return false;
    *result = s->backend.init[channel];
    return true;
}
