// SPDX-License-Identifier: GPL-3.0-or-later
#include "esc_uart.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_ll.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"
#include "sdkconfig.h"
#include <string.h>

#define ESC_UART UART_NUM_1
static portMUX_TYPE tx_lock = portMUX_INITIALIZER_UNLOCKED;

bool esc_uart_pin_allowed(void *ctx, int pin)
{
    (void)ctx;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) return false;
#if CONFIG_IDF_TARGET_ESP32C3
    if (pin >= 12 && pin <= 19) return false; // Flash and native USB.
#elif CONFIG_IDF_TARGET_ESP32S3
    if (pin == 19 || pin == 20 || (pin >= 26 && pin <= 37)) return false;
#else
#error "MVP supports ESP32-C3 and ESP32-S3 only"
#endif
    return true;
}

static void release_wire(void *ctx)
{
    esc_uart_t *link = ctx;
    for (int i = 0; i < link->count; ++i) gpio_set_direction(link->pins[i], GPIO_MODE_INPUT);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, U1RXD_IN_IDX, false);
}

static bool select_wire(void *ctx, uint8_t channel)
{
    esc_uart_t *link = ctx;
    if (channel >= link->count) return false;
    release_wire(ctx);
    link->selected = channel;
    uart_flush_input(ESC_UART);
    esp_rom_gpio_connect_in_signal(link->pins[channel], U1RXD_IN_IDX, false);
    return true;
}

static bool send_wire(void *ctx, const uint8_t *data, size_t size)
{
    esc_uart_t *link = ctx;
    if (link->selected < 0 || !size || size > 258) return false;
    int pin = link->pins[link->selected];
    uart_flush_input(ESC_UART);
    // Disconnect RX during transmission to suppress echo.
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, U1RXD_IN_IDX, false);
    gpio_set_level(pin, 1);
    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(pin, U1TXD_OUT_IDX, false, false);
    uart_dev_t *hw = UART_LL_GET_HW(ESC_UART);
    int64_t deadline = esp_timer_get_time() + (int64_t)size * 10 * 1000000 / 19200 + 20000;
    size_t prefix = size > 23 ? size - 16 : 0;
    size_t sent = 0;
    // Send the prefix with interrupts enabled.
    // Send the tail in a critical section for fast GPIO turnaround.
    while (sent < prefix && esp_timer_get_time() < deadline) {
        size_t available = uart_ll_get_txfifo_len(hw);
        size_t chunk = prefix - sent;
        if (chunk > available) chunk = available;
        if (chunk) { uart_ll_write_txfifo(hw, data + sent, chunk); sent += chunk; }
    }
    while (!uart_ll_is_tx_idle(hw) && esp_timer_get_time() < deadline) { }
    // Limit the critical section to 23 bytes (about 12 ms).
    portENTER_CRITICAL(&tx_lock);
    bool ok = sent == prefix && uart_ll_is_tx_idle(hw) && esp_timer_get_time() < deadline;
    if (ok) uart_ll_write_txfifo(hw, data + sent, size - sent);
    while (!uart_ll_is_tx_idle(hw) && esp_timer_get_time() < deadline) { }
    ok = ok && uart_ll_is_tx_idle(hw);
    if (!ok) uart_ll_txfifo_rst(hw);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    esp_rom_gpio_connect_in_signal(pin, U1RXD_IN_IDX, false);
    portEXIT_CRITICAL(&tx_lock);
    return ok;
}

static bool receive_wire(void *ctx, uint8_t *data, size_t size, unsigned timeout_ms)
{
    (void)ctx;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    size_t received = 0;
    while (received < size) {
        int64_t remaining = deadline - esp_timer_get_time();
        if (remaining <= 0) return false;
        TickType_t ticks = pdMS_TO_TICKS((remaining + 999) / 1000);
        if (!ticks) ticks = 1;
        int got = uart_read_bytes(ESC_UART, data + received, size - received, ticks);
        if (got < 0) return false;
        received += got;
    }
    return true;
}

static void pulse_wire(void *ctx, unsigned duration_ms)
{
    esc_uart_t *link = ctx;
    if (link->selected < 0) return;
    int pin = link->pins[link->selected];
    gpio_set_level(pin, 0);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_direction(pin, GPIO_MODE_INPUT);
}

esp_err_t esc_uart_init(esc_uart_t *link, const int pins[4], int count)
{
    memset(link, 0, sizeof(*link));
    memcpy(link->pins, pins, sizeof(link->pins));
    link->count = count;
    link->selected = -1;
    for (int i = 0; i < count; ++i) {
        gpio_config_t config = {.pin_bit_mask = 1ULL << pins[i], .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};
        esp_err_t error = gpio_config(&config);
        if (error != ESP_OK) return error;
    }
    uart_config_t config = {.baud_rate = 19200, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    esp_err_t error = uart_driver_install(ESC_UART, 2048, 0, 0, NULL, 0);
    if (error != ESP_OK) return error;
    if ((error = uart_param_config(ESC_UART, &config)) != ESP_OK) return error;
    if ((error = uart_set_rx_full_threshold(ESC_UART, 1)) != ESP_OK) return error;
    if ((error = uart_set_rx_timeout(ESC_UART, 1)) != ESP_OK) return error;
    release_wire(link);
    return ESP_OK;
}

esc_wire_t esc_uart_wire(esc_uart_t *link)
{
    return (esc_wire_t){.ctx = link, .select = select_wire, .send = send_wire,
        .receive = receive_wire, .release = release_wire, .pulse_low = pulse_wire};
}
