// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { PT_MAX_CHANNELS = 4, PT_MAX_PAYLOAD = 256, PT_MAX_REQUEST = 263 };
enum { PT_OK = 0, PT_INVALID_CMD = 2, PT_INVALID_CRC = 3,
       PT_INVALID_CHANNEL = 8, PT_INVALID_PARAM = 9, PT_DEVICE_ERROR = 15 };

typedef struct {
    void *ctx;
    bool (*connect)(void *ctx, uint8_t channel, uint8_t info[4]);
    bool (*read)(void *ctx, uint16_t address, size_t size, uint8_t *data);
    bool (*erase_page)(void *ctx, uint8_t page);
    bool (*write)(void *ctx, uint16_t address, size_t size, const uint8_t *data);
    bool (*alive)(void *ctx);
    bool (*reset)(void *ctx, uint8_t channel, bool reboot);
    void (*release)(void *ctx);
} pt_esc_ops_t;

typedef bool (*pt_output_fn)(void *ctx, const uint8_t *data, size_t size);
typedef bool (*pt_pin_allowed_fn)(void *ctx, int pin);

typedef struct {
    pt_esc_ops_t esc;
    pt_output_fn output;
    void *output_ctx;
    uint8_t channels;
    uint8_t uid[12];
    char board[4];
    bool fourway;
    bool connected;
    uint8_t selected;
    uint8_t mode;
    uint8_t input[PT_MAX_REQUEST];
    size_t used;
    uint32_t last_byte_ms;
} pt_t;

uint16_t pt_crc_xmodem(const uint8_t *data, size_t size);
int pt_validate_pins(const int pins[PT_MAX_CHANNELS], pt_pin_allowed_fn allowed, void *ctx);
void pt_init(pt_t *pt, uint8_t channels, const pt_esc_ops_t *esc,
             pt_output_fn output, void *output_ctx, const uint8_t uid[12], const char board[4]);
void pt_feed(pt_t *pt, const uint8_t *data, size_t size, uint32_t now_ms);
void pt_expire(pt_t *pt, uint32_t now_ms);
