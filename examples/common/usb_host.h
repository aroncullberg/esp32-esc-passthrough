// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "esc_passthrough.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>

static bool usb_output(void *ctx, const uint8_t *data, size_t size)
{
    (void)ctx;
    size_t sent = 0;
    while (sent < size) {
        int count = usb_serial_jtag_write_bytes(data + sent, size - sent, pdMS_TO_TICKS(100));
        if (count <= 0) return false;
        sent += count;
    }
    return usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100)) == ESP_OK;
}

static esc_passthrough_config_t usb_config(const int pins[PT_MAX_CHANNELS])
{
    esp_log_level_set("*", ESP_LOG_NONE);
    usb_serial_jtag_driver_config_t usb = {.tx_buffer_size = 2048, .rx_buffer_size = 2048};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    esc_passthrough_config_t config = esc_passthrough_config_default();
    memcpy(config.pins, pins, sizeof(config.pins));
    config.output = usb_output;
    ESP_ERROR_CHECK(esp_read_mac(config.uid, ESP_MAC_BASE));
    memcpy(config.uid + 6, "ESC0.3", 6);
#if CONFIG_IDF_TARGET_ESP32S3
    memcpy(config.board, "ESPS", 4);
#endif
    return config;
}

static bool usb_pump(esc_passthrough_t *session, bool *was_connected)
{
    bool connected = usb_serial_jtag_is_connected();
    bool lost = *was_connected && !connected;
    if (lost) esc_passthrough_transport_lost(session);
    *was_connected = connected;
    uint8_t input[512];
    if (connected) {
        int size = usb_serial_jtag_read_bytes(input, sizeof(input), pdMS_TO_TICKS(10));
        if (size > 0) esc_passthrough_feed(session, input, size,
            (uint32_t)(esp_timer_get_time() / 1000));
    } else {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    esc_passthrough_poll(session, (uint32_t)(esp_timer_get_time() / 1000));
    return lost;
}
