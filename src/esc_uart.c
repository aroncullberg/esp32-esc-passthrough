// SPDX-License-Identifier: GPL-3.0-or-later
#include "esc_uart.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_ll.h"
#include "hal/uart_periph.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"
#include "sdkconfig.h"
#include <string.h>

static portMUX_TYPE tx_lock = portMUX_INITIALIZER_UNLOCKED;

bool esc_uart_pin_allowed(void *ctx, int pin)
{
    (void)ctx;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) return false;
#if CONFIG_IDF_TARGET_ESP32C3
    if (pin >= 12 && pin <= 19) return false; // Flash and native USB.
#elif CONFIG_IDF_TARGET_ESP32S3
    if (pin == 19 || pin == 20 || (pin >= 26 && pin <= 37)) return false;
#endif
    return true;
}

static void remember(esc_uart_t *link, esp_err_t error)
{
    if (error != ESP_OK) link->io_error = error;
}

static unsigned rx_signal(const esc_uart_t *link)
{
    return UART_PERIPH_SIGNAL(link->port, SOC_UART_PERIPH_SIGNAL_RX);
}

static void detach_output(esc_uart_t *link, int pin)
{
    // ROM routing enables output, so input mode must be set afterward.
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    remember(link, gpio_set_direction(pin, GPIO_MODE_INPUT));
}

static void release_wire(void *ctx)
{
    esc_uart_t *link = ctx;
    if (!link->installed) return;
    // Mask RX before changing routes to avoid spurious start bits.
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, rx_signal(link), false);
    for (int i = 0; i < link->count; ++i) {
        if (!(link->claimed_mask & (1u << i))) continue;
        detach_output(link, link->pins[i]);
        if (link->active && CONFIG_ESC_PASSTHROUGH_BOOT_WAIT_MS) {
            // AM32's bootloader pull-down can overpower the ESP pull-up.
            remember(link, gpio_set_level(link->pins[i], 1));
            remember(link, gpio_set_direction(link->pins[i], GPIO_MODE_OUTPUT));
        }
    }
    link->selected = -1;
}

static bool select_wire(void *ctx, uint8_t channel)
{
    esc_uart_t *link = ctx;
    if (!link->active || channel >= link->count || link->io_error != ESP_OK) return false;
    int64_t remaining = link->boot_ready_us - esp_timer_get_time();
    if (remaining > 0) {
        // Wait here so MSP passthrough entry can reply immediately.
        vTaskDelay(pdMS_TO_TICKS((remaining + 999) / 1000) + 1);
    }
    release_wire(ctx);
    remember(link, uart_flush_input(link->port));
    remember(link, gpio_set_direction(link->pins[channel], GPIO_MODE_INPUT));
    if (link->io_error != ESP_OK) return false;
    link->selected = channel;
    esp_rom_gpio_connect_in_signal(link->pins[channel], rx_signal(link), false);
    return true;
}

static bool send_wire(void *ctx, const uint8_t *data, size_t size)
{
    esc_uart_t *link = ctx;
    if (!link->active || link->selected < 0 || !data || !size || size > 258 ||
        link->io_error != ESP_OK) return false;
    int pin = link->pins[link->selected];
    remember(link, uart_flush_input(link->port));
    // Disconnect RX during transmission to suppress echo.
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, rx_signal(link), false);
    remember(link, gpio_set_level(pin, 1));
    remember(link, gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT));
    if (link->io_error != ESP_OK) { release_wire(link); return false; }
    esp_rom_gpio_connect_out_signal(pin,
        UART_PERIPH_SIGNAL(link->port, SOC_UART_PERIPH_SIGNAL_TX), false, false);
    uart_dev_t *hw = UART_LL_GET_HW(link->port);
    int64_t deadline = esp_timer_get_time() + (int64_t)size * 10 * 1000000 / 19200 + 20000;
    size_t prefix = size > 23 ? size - 16 : 0;
    size_t sent = 0;
    // Protect the final TX-to-RX turnaround from task preemption.
    while (sent < prefix && esp_timer_get_time() < deadline) {
        size_t available = uart_ll_get_txfifo_len(hw);
        size_t chunk = prefix - sent;
        if (chunk > available) chunk = available;
        if (chunk) { uart_ll_write_txfifo(hw, data + sent, chunk); sent += chunk; }
    }
    while (!uart_ll_is_tx_idle(hw) && esp_timer_get_time() < deadline) { }
    portENTER_CRITICAL(&tx_lock);
    bool ok = sent == prefix && uart_ll_is_tx_idle(hw) && esp_timer_get_time() < deadline;
    if (ok) uart_ll_write_txfifo(hw, data + sent, size - sent);
    while (!uart_ll_is_tx_idle(hw) && esp_timer_get_time() < deadline) { }
    ok = ok && uart_ll_is_tx_idle(hw);
    if (!ok) uart_ll_txfifo_rst(hw);
    detach_output(link, pin);
    esp_rom_gpio_connect_in_signal(pin, rx_signal(link), false);
    portEXIT_CRITICAL(&tx_lock);
    return ok && link->io_error == ESP_OK;
}

static bool receive_wire(void *ctx, uint8_t *data, size_t size, unsigned timeout_ms)
{
    esc_uart_t *link = ctx;
    link->received = 0;
    if (!link->active || link->selected < 0 || link->io_error != ESP_OK) return false;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    size_t received = 0;
    while (received < size) {
        int64_t remaining = deadline - esp_timer_get_time();
        if (remaining <= 0) return false;
        TickType_t ticks = pdMS_TO_TICKS((remaining + 999) / 1000);
        if (!ticks) ticks = 1;
        int got = uart_read_bytes(link->port, data + received, size - received, ticks);
        if (got < 0) return false;
        received += got;
        link->received = received;
    }
    return true;
}

static size_t received_size(void *ctx)
{
    return ((esc_uart_t *)ctx)->received;
}

static bool pulse_wire(void *ctx, unsigned duration_ms)
{
    esc_uart_t *link = ctx;
    if (!link->active || link->selected < 0 || link->io_error != ESP_OK) return false;
    int pin = link->pins[link->selected];
    // Keep reset pulses out of the UART RX buffer.
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, rx_signal(link), false);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    remember(link, gpio_set_level(pin, 0));
    remember(link, gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    if (link->io_error == ESP_OK) {
        TickType_t ticks = pdMS_TO_TICKS(duration_ms) + 1;
        vTaskDelay(ticks);
    }
    remember(link, gpio_set_direction(pin, GPIO_MODE_INPUT));
    return link->io_error == ESP_OK;
}

esp_err_t esc_uart_init(esc_uart_t *link, const int pins[PT_MAX_CHANNELS], int count, int port)
{
    if (!link || !pins || count < 1 || count > PT_MAX_CHANNELS ||
        port < 0 || port >= SOC_UART_HP_NUM) return ESP_ERR_INVALID_ARG;
    memset(link, 0, sizeof(*link));
    memcpy(link->pins, pins, sizeof(link->pins));
    link->count = count;
    link->selected = -1;
    link->port = port;
    return ESP_OK;
}

esp_err_t esc_uart_start(void *ctx)
{
    esc_uart_t *link = ctx;
    if (link->active) return ESP_OK;
    if (link->installed || link->claimed_mask) return ESP_ERR_INVALID_STATE;
    if (uart_is_driver_installed(link->port)) return ESP_ERR_INVALID_STATE;
    esp_err_t error = uart_driver_install(link->port, 2048, 0, 0, NULL, 0);
    if (error != ESP_OK) return error;
    link->installed = true;
    link->io_error = ESP_OK;
    uart_config_t config = {.baud_rate = 19200, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    if ((error = uart_param_config(link->port, &config)) != ESP_OK) return error;
    if ((error = uart_set_rx_full_threshold(link->port, 1)) != ESP_OK) return error;
    if ((error = uart_set_rx_timeout(link->port, 1)) != ESP_OK) return error;
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, rx_signal(link), false);
    for (int i = 0; i < link->count; ++i) {
        link->claimed_mask |= 1u << i;
        if ((error = gpio_set_level(link->pins[i], 1)) != ESP_OK) return error;
        detach_output(link, link->pins[i]);
        if (link->io_error != ESP_OK) return link->io_error;
        gpio_config_t gpio = {.pin_bit_mask = 1ULL << link->pins[i],
            .mode = CONFIG_ESC_PASSTHROUGH_BOOT_WAIT_MS ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};
        if ((error = gpio_config(&gpio)) != ESP_OK) return error;
    }
    link->boot_ready_us = esp_timer_get_time() + (int64_t)CONFIG_ESC_PASSTHROUGH_BOOT_WAIT_MS * 1000;
    link->active = true;
    release_wire(link);
    if (link->io_error != ESP_OK) {
        link->active = false;
        return link->io_error;
    }
    return ESP_OK;
}

esp_err_t esc_uart_stop(void *ctx)
{
    esc_uart_t *link = ctx;
    link->active = false;
    link->boot_ready_us = 0;
    link->io_error = ESP_OK;
    release_wire(link);
    esp_err_t error = link->io_error;
    if (link->installed) {
        esp_err_t deleted = uart_driver_delete(link->port);
        if (deleted != ESP_OK) return deleted;
        link->installed = false;
    }
    for (int i = 0; i < link->count; ++i) {
        if (!(link->claimed_mask & (1u << i))) continue;
        detach_output(link, link->pins[i]);
        if (link->io_error != ESP_OK) error = link->io_error;
        esp_err_t reset = gpio_reset_pin(link->pins[i]);
        if (reset == ESP_OK) link->claimed_mask &= ~(1u << i);
        else error = reset;
    }
    link->selected = -1;
    return error;
}

esc_wire_t esc_uart_wire(esc_uart_t *link)
{
    return (esc_wire_t){.ctx = link, .select = select_wire, .send = send_wire,
        .receive = receive_wire, .release = release_wire, .pulse_low = pulse_wire,
        .received_size = received_size};
}
