// SPDX-License-Identifier: GPL-3.0-or-later
#include "passthrough.h"
#include "esc_backend.h"
#include "esc_uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>

static pt_t protocol;
static esc_uart_t link;
static esc_backend_t backend;

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

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);
    const int pins[] = {CONFIG_ESC1_GPIO, CONFIG_ESC2_GPIO, CONFIG_ESC3_GPIO, CONFIG_ESC4_GPIO};
    int count = pt_validate_pins(pins, esc_uart_pin_allowed, NULL);
    if (count < 0) return;
    ESP_ERROR_CHECK(esc_uart_init(&link, pins, count));
    usb_serial_jtag_driver_config_t usb = {.tx_buffer_size = 2048, .rx_buffer_size = 2048};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    esc_wire_t wire = esc_uart_wire(&link);
    esc_backend_init(&backend, &wire);
    pt_esc_ops_t ops = esc_backend_ops(&backend);
    uint8_t uid[12] = {0};
    ESP_ERROR_CHECK(esp_read_mac(uid, ESP_MAC_BASE));
    memcpy(uid + 6, "ESC0.1", 6);
#if CONFIG_IDF_TARGET_ESP32C3
    const char board[] = "ESP3";
#else
    const char board[] = "ESPS";
#endif
    pt_init(&protocol, count, &ops, usb_output, NULL, uid, board);
    uint8_t input[512];
    for (;;) {
        int size = usb_serial_jtag_read_bytes(input, sizeof(input), pdMS_TO_TICKS(10));
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (size > 0) pt_feed(&protocol, input, size, now_ms);
        else pt_expire(&protocol, now_ms);
    }
}
