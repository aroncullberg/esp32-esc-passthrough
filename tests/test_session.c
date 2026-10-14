// SPDX-License-Identifier: GPL-3.0-or-later
#include "esc_passthrough.h"
#include "esc_uart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

bool esc_uart_pin_allowed(void *ctx, int pin) { (void)ctx; return pin >= 0 && pin < 12; }
esp_err_t esc_uart_init(esc_uart_t *link, const int pins[4], int count, int port)
{ (void)link; (void)pins; (void)count; return port == 1 ? ESP_OK : ESP_ERR_INVALID_ARG; }
esp_err_t esc_uart_start(void *ctx) { (void)ctx; assert(false); return ESP_FAIL; }
esp_err_t esc_uart_stop(void *ctx) { (void)ctx; assert(false); return ESP_FAIL; }
esc_wire_t esc_uart_wire(esc_uart_t *link) { (void)link; return (esc_wire_t){0}; }

static esc_passthrough_t *session;
static char events[4096];
static unsigned event_count, acquisitions, releases, starts, stops;
static bool acquire_ok, release_ok, start_ok, stop_ok, output_ok, active, motors_stopped;
static uint8_t no_reply_mask, pulse_fail_mask, reset_fail_mask, selected;
static uint8_t reset_mask, pulse_mask, init_mask, response[264];
static size_t sent_size, response_size;
static esc_session_end_t end_result;
static void event(char c) { assert(event_count < sizeof(events) - 1); events[event_count++] = c; events[event_count] = 0; }
static esp_err_t acquire(void *ctx)
{
    (void)ctx; event('A'); ++acquisitions;
    motors_stopped = true;
    return acquire_ok ? ESP_OK : ESP_FAIL;
}
static esp_err_t release(void *ctx, const esc_session_end_t *result)
{
    (void)ctx; event('R'); ++releases;
    assert(motors_stopped);
    assert(!active || !result->io_released);
    end_result = *result;
    return release_ok ? ESP_OK : ESP_FAIL;
}
static esp_err_t start(void *ctx)
{
    (void)ctx; event('S'); ++starts; assert(motors_stopped);
    // A failed start can still have acquired IO and must be stopped.
    active = true;
    return start_ok ? ESP_OK : ESP_FAIL;
}
static esp_err_t stop(void *ctx)
{
    (void)ctx; event('D'); ++stops;
    if (stop_ok) active = false;
    return stop_ok ? ESP_OK : ESP_FAIL;
}
static bool output(void *ctx, const uint8_t *data, size_t size)
{
    (void)ctx; event('O');
    assert(size <= sizeof(response));
    memcpy(response, data, size); response_size = size;
    if (data[0] == '$' && data[2] == '>' && (data[4] == 1 || data[4] == 245)) {
        assert(motors_stopped);
        if (data[4] == 245) assert(active);
    }
    return output_ok;
}
static bool select_wire(void *ctx, uint8_t channel)
{
    (void)ctx; assert(active); event('s'); selected = channel; return true;
}
static bool send_wire(void *ctx, const uint8_t *data, size_t size)
{
    (void)ctx; (void)data; assert(active);
    sent_size = size;
    if (size == 17) { event('b'); init_mask |= 1u << selected; }
    if (size == 4 && data[0] == 0) {
        event('r'); reset_mask |= 1u << selected;
        return !(reset_fail_mask & (1u << selected));
    }
    return true;
}
static bool receive_wire(void *ctx, uint8_t *data, size_t size, unsigned timeout)
{
    (void)ctx; assert(active && timeout);
    if (no_reply_mask & (1u << selected)) return false;
    if (sent_size == 17) {
        assert(size == 9);
        memcpy(data, (uint8_t[]){'4','7','1','d',0xe8,0xb2,6,1,0x30}, 9);
        return true;
    }
    if (size == 1) { data[0] = 0xc1; return true; }
    return false;
}
static bool pulse_wire(void *ctx, unsigned ms)
{
    (void)ctx; assert(active && ms == 300); event('p');
    pulse_mask |= 1u << selected;
    return !(pulse_fail_mask & (1u << selected));
}
static void release_wire(void *ctx) { (void)ctx; assert(active); event('w'); }

static void setup(void)
{
    assert(!session);
    event_count = acquisitions = releases = starts = stops = 0;
    no_reply_mask = pulse_fail_mask = reset_fail_mask = 0;
    reset_mask = pulse_mask = init_mask = 0;
    acquire_ok = release_ok = start_ok = stop_ok = output_ok = true;
    active = motors_stopped = false;
    events[0] = 0;
    esc_passthrough_io_t io = {.start = start, .stop = stop,
        .wire = {.select = select_wire, .send = send_wire, .receive = receive_wire,
                 .pulse_low = pulse_wire, .release = release_wire}};
    esc_passthrough_config_t config = esc_passthrough_config_default();
    config.pins[0] = 0; config.pins[1] = 1;
    config.io = &io; config.output = output;
    config.lifecycle = (esc_passthrough_lifecycle_t){.acquire = acquire, .release = release};
    config.session_timeout_ms = 1000;
    assert(esc_passthrough_create(&config, &session) == ESP_OK);
    assert(event_count == 0 && !esc_passthrough_is_held(session));
}
static void teardown(void)
{
    stop_ok = true;
    assert(esc_passthrough_destroy(session) == ESP_OK);
    session = NULL;
}
static size_t msp(uint8_t *out, uint8_t cmd, const uint8_t *payload, unsigned size)
{
    memcpy(out, "$M<", 3); out[3] = size; out[4] = cmd;
    uint8_t crc = size ^ cmd;
    for (unsigned i = 0; i < size; ++i) { out[5+i] = payload[i]; crc ^= payload[i]; }
    out[5+size] = crc; return 6+size;
}
static void discovery(uint32_t now)
{
    uint8_t bytes[6]; esc_passthrough_feed(session, bytes, msp(bytes, 1, NULL, 0), now);
}
static void enter(uint32_t now)
{
    uint8_t bytes[6]; esc_passthrough_feed(session, bytes, msp(bytes, 245, NULL, 0), now);
}
static size_t fourway(uint8_t *out, uint8_t cmd, uint16_t address, uint8_t param)
{
    memcpy(out, (uint8_t[]){0x2f,cmd,address >> 8,address & 255,1,param}, 6);
    uint16_t crc = pt_crc_xmodem(out, 6); out[6] = crc >> 8; out[7] = crc; return 8;
}
static void request(uint8_t cmd, uint16_t address, uint8_t param)
{
    uint8_t bytes[8]; esc_passthrough_feed(session, bytes, fourway(bytes, cmd, address, param), 100);
}
static uint8_t ack(void) { return response[response_size - 3]; }

static void test_validation_and_discovery(void)
{
    setup();
    uint8_t packet[8];
    size_t size = msp(packet, 1, NULL, 0); packet[size-1] ^= 1;
    esc_passthrough_feed(session, packet, size, 0); assert(event_count == 0);
    size = msp(packet, 1, (uint8_t[]){0}, 1);
    esc_passthrough_feed(session, packet, size, 0); assert(response[2] == '!' && !acquisitions);
    size = msp(packet, 245, (uint8_t[]){0}, 1);
    esc_passthrough_feed(session, packet, size, 0); assert(response[2] == '!' && !acquisitions);
    size = msp(packet, 101, NULL, 0);
    esc_passthrough_feed(session, packet, size, 0); assert(!acquisitions);
    unsigned before = event_count;
    discovery(1); assert(!strcmp(events + before, "AO"));
    assert(esc_passthrough_is_held(session) && !starts);
    discovery(2); assert(acquisitions == 1 && !starts);
    request(0x35, 1, 0); assert(ack() == PT_DEVICE_ERROR && !starts && !reset_mask);
    request(0x37, 0, 0); assert(ack() == PT_DEVICE_ERROR && !init_mask);
    request(0x34, 0, 0);
    assert(releases == 1 && !stops && end_result.io_released && !end_result.needs_reboot_mask);
    assert(end_result.reason == ESC_SESSION_NORMAL_EXIT && !esc_passthrough_is_held(session));
    request(0x34, 0, 0); assert(releases == 1);
    teardown(); assert(releases == 1);
}

static void test_entry_reconnect_and_failure(void)
{
    setup(); enter(0);
    assert(!strncmp(events, "ASO", 3) && starts == 1 && acquisitions == 1);
    request(0x37, 0, 0); assert(ack() == PT_OK);
    uint8_t packet[8]; size_t size = msp(packet, 1, (uint8_t[]){0}, 1);
    esc_passthrough_feed(session, packet, size, 100);
    request(0x30, 0, 0); assert(ack() == PT_OK);
    discovery(200); assert(releases == 0 && acquisitions == 1 && starts == 1);
    request(0x37, 0, 1); assert(ack() == PT_DEVICE_ERROR);
    enter(250); enter(300); assert(starts == 1 && acquisitions == 1);
    request(0x34, 0, 0);
    assert(end_result.bootloader_attempted_mask == 1 && end_result.reboot_sent_mask == 1);
    assert(!end_result.needs_reboot_mask && stops == 1 && releases == 1);
    teardown();

    setup(); acquire_ok = false; enter(0);
    assert(response[2] == '!' && starts == 0 && releases == 0 && !esc_passthrough_is_held(session));
    teardown(); assert(releases == 0);
    setup(); start_ok = false; enter(0);
    assert(response[2] == '!' && stops == 1 && releases == 1 && !active);
    assert(end_result.reason == ESC_SESSION_SETUP_FAILURE);
    teardown();
    setup(); output_ok = false; discovery(0);
    assert(releases == 1 && !starts && end_result.reason == ESC_SESSION_REPLY_FAILURE);
    teardown();
    setup(); output_ok = false; enter(0);
    assert(stops == 1 && releases == 1 && end_result.reason == ESC_SESSION_REPLY_FAILURE);
    teardown();
    setup(); output_ok = false;
    uint8_t coalesced[12];
    msp(coalesced, 1, NULL, 0); msp(coalesced + 6, 245, NULL, 0);
    esc_passthrough_feed(session, coalesced, sizeof(coalesced), 0);
    assert(acquisitions == 1 && releases == 1 && !starts);
    teardown();
}

static void test_reboot_tracking(void)
{
    setup(); enter(0); no_reply_mask = 2;
    request(0x37, 0, 0); assert(ack() == PT_OK);
    request(0x3a, 0, 1); assert(ack() == PT_DEVICE_ERROR && releases == 0);
    request(0x37, 0, 1); assert(ack() == PT_DEVICE_ERROR && releases == 0);
    request(0x35, 0, 0); assert(ack() == PT_OK);
    request(0x35, 1, 0); assert(ack() == PT_OK && pulse_mask == 1);
    unsigned before = event_count;
    request(0x34, 0, 0);
    assert(end_result.bootloader_attempted_mask == 3 && end_result.reboot_sent_mask == 3);
    assert(!end_result.needs_reboot_mask && pulse_mask == 3);
    // Reply -> reboot -> IO teardown -> owner release.
    const char *tail = events + before;
    assert(tail[0] == 'O' && strchr(tail, 'p') < strchr(tail, 'D') && strchr(tail, 'D') < strchr(tail, 'R'));
    teardown();

    setup(); enter(0); request(0x37, 0, 0); request(0x35, 1, 0);
    request(0x37, 0, 0); pulse_fail_mask = 1;
    request(0x37, 0, 1); request(0x34, 0, 0);
    assert(end_result.needs_reboot_mask == 1 && end_result.reboot_sent_mask == 3);
    assert(pulse_mask == 3 && end_result.io_released);
    esc_session_end_t saved; assert(esc_passthrough_last_end(session, &saved));
    assert(saved.needs_reboot_mask == 1);
    pulse_fail_mask = 0;
    discovery(500);
    assert(starts == 1);
    request(0x34, 0, 0);
    assert(starts == 2 && stops == 2 && releases == 2 && end_result.needs_reboot_mask == 0);
    assert(end_result.bootloader_attempted_mask == 1 && end_result.reboot_sent_mask == 1);
    teardown();

    setup(); enter(0); request(0x37, 0, 0); reset_fail_mask = 1;
    request(0x34, 0, 0);
    assert(end_result.needs_reboot_mask == 1 && pulse_mask == 1);
    teardown();
}

static void test_abnormal_end_and_cleanup(void)
{
    setup(); enter(0); request(0x37, 0, 0); output_ok = false;
    request(0x34, 0, 0);
    assert(releases == 1 && stops == 1 && pulse_mask == 1);
    assert(end_result.reason == ESC_SESSION_REPLY_FAILURE && end_result.io_released);
    teardown();
    setup(); enter(0); request(0x37, 0, 0);
    esc_passthrough_transport_lost(session);
    assert(releases == 1 && stops == 1 && pulse_mask == 1);
    assert(end_result.reason == ESC_SESSION_TRANSPORT_LOSS);
    esc_passthrough_transport_lost(session); assert(releases == 1);
    teardown();
    setup(); discovery(UINT32_MAX - 500);
    esc_passthrough_poll(session, 498); assert(releases == 0);
    esc_passthrough_poll(session, 499); assert(releases == 1 && end_result.reason == ESC_SESSION_TIMEOUT);
    teardown();
    setup(); discovery(0); discovery(900);
    esc_passthrough_poll(session, 1000); assert(releases == 0);
    esc_passthrough_shutdown(session); assert(releases == 1 && end_result.reason == ESC_SESSION_SHUTDOWN);
    teardown();

    setup(); enter(0); request(0x37, 0, 0); stop_ok = false;
    request(0x34, 0, 0);
    assert(releases == 1 && !end_result.io_released && active);
    discovery(500); assert(acquisitions == 1 && response[2] == '!');
    assert(esc_passthrough_destroy(session) != ESP_OK && releases == 1 && active);
    stop_ok = true;
    assert(esc_passthrough_retry_cleanup(session) == ESP_OK && !active);
    esc_session_end_t saved; assert(esc_passthrough_last_end(session, &saved) && saved.io_released);
    discovery(600); assert(acquisitions == 2);
    request(0x34, 0, 0); assert(releases == 2);
    teardown();
    setup(); enter(0); release_ok = false;
    assert(esc_passthrough_destroy(session) == ESP_FAIL && releases == 1 && !active);
    release_ok = true;
    teardown(); assert(releases == 1);
}

static void test_configure_standalone(void)
{
    esc_passthrough_config_t config = esc_passthrough_config_default();
    assert(esc_passthrough_create(&config, &session) == ESP_ERR_INVALID_ARG && !session);
    config.pins[0] = 0; config.output = output;
    config.lifecycle.acquire = acquire;
    assert(esc_passthrough_create(&config, &session) == ESP_ERR_INVALID_ARG);
    config.lifecycle.acquire = NULL;
    esc_passthrough_io_t io = {.start = start, .stop = stop,
        .wire = {.select = select_wire, .send = send_wire, .receive = receive_wire,
                 .pulse_low = pulse_wire, .release = release_wire}};
    config.io = &io;
    io.wire.pulse_low = NULL;
    assert(esc_passthrough_create(&config, &session) == ESP_ERR_INVALID_ARG);
    io.wire.pulse_low = pulse_wire;
    unsigned acquired_before = acquisitions, released_before = releases;
    assert(esc_passthrough_create(&config, &session) == ESP_OK);
    motors_stopped = true;
    enter(0); request(0x37, 0, 0); request(0x34, 0, 0);
    assert(acquisitions == acquired_before && releases == released_before);
    esc_session_end_t report;
    assert(esc_passthrough_last_end(session, &report) && report.io_released && !report.needs_reboot_mask);
    teardown();
}

static void test_retained_diagnostics(void)
{
    setup();
    uint8_t packet[8];
    esc_init_diagnostic_t diagnostic;
    assert(!esc_passthrough_init_diagnostic(session, 0, &diagnostic));
    esc_passthrough_feed(session, packet, msp(packet, 253, (uint8_t[]){0}, 1), 0);
    assert(response[2] == '>' && !acquisitions && !starts && event_count == 1);
    assert(!memcmp(response + 5, "ESC init ch=0 gpio=0 attempts=0\n", response[3]));
    enter(1); request(0x37, 0, 0); assert(ack() == PT_OK);
    assert(esc_passthrough_init_diagnostic(session, 0, &diagnostic));
    assert(diagnostic.attempts == 1 && diagnostic.attempt[0].result == ESC_INIT_OK);
    assert(diagnostic.attempt[0].received == 9 && !memcmp(diagnostic.attempt[0].reply, "471d", 4));
    no_reply_mask = 2; request(0x37, 0, 1); assert(ack() == PT_DEVICE_ERROR);
    request(0x34, 0, 0);
    unsigned before = event_count, starts_before = starts, acquisitions_before = acquisitions;
    esc_passthrough_feed(session, packet, msp(packet, 253, (uint8_t[]){1}, 1), 2);
    assert(response[2] == '>' && starts == starts_before && acquisitions == acquisitions_before);
    assert(event_count == before + 1 && events[before] == 'O' && !active);
    assert(esc_passthrough_init_diagnostic(session, 1, &diagnostic));
    assert(diagnostic.attempts == 3 && diagnostic.attempt[2].result == ESC_INIT_RX_TIMEOUT);
    assert(diagnostic.attempt[2].received == ESC_RX_SIZE_UNKNOWN);
    assert(!esc_passthrough_init_diagnostic(session, 2, &diagnostic));
    esc_passthrough_feed(session, packet, msp(packet, 253, (uint8_t[]){2}, 1), 3);
    assert(response[2] == '!' && !response[3] && starts == starts_before);
    teardown();
}

int main(void)
{
    test_validation_and_discovery();
    test_entry_reconnect_and_failure();
    test_reboot_tracking();
    test_abnormal_end_and_cleanup();
    test_configure_standalone();
    test_retained_diagnostics();
    puts("PASS: validated/lazy entry, exactly-once callbacks, reconnects, multi-channel reboot debt, abnormal ends and cleanup retry");
    return 0;
}
