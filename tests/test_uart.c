// SPDX-License-Identifier: GPL-3.0-or-later
#include "esc_uart.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "hal/uart_ll.h"
#include "hal/uart_periph.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"
#include "sdkconfig.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

uart_dev_t fake_hw;
static bool driver_installed, foreign_driver, install_ok, param_ok, delete_ok, idle;
static int config_fail_pin, reset_fail_pin, selected_rx;
static unsigned installs, deletes, pin_calls, gpio_out[22], rx_route[2], directions[22], levels[22];
static unsigned uart_port_seen, fifo_sent;
static unsigned boot_waits;
static int64_t now_us;
static uint8_t rx_bytes[9];
static size_t rx_size, rx_offset, rx_chunk;
bool uart_is_driver_installed(int port) { uart_port_seen = port; return driver_installed; }
esp_err_t uart_driver_install(int port, int rx, int tx, int queue_size, void *queue, int flags)
{
    (void)rx; (void)tx; (void)queue_size; (void)queue; (void)flags;
    uart_port_seen = port; ++installs;
    if (!install_ok) return ESP_FAIL;
    assert(!driver_installed); driver_installed = true; return ESP_OK;
}
esp_err_t uart_driver_delete(int port)
{
    uart_port_seen = port; ++deletes; assert(driver_installed && !foreign_driver);
    if (!delete_ok) return ESP_FAIL;
    driver_installed = false; return ESP_OK;
}
esp_err_t uart_param_config(int port, const uart_config_t *config)
{
    uart_port_seen = port; assert(config->baud_rate == 19200);
    return param_ok ? ESP_OK : ESP_FAIL;
}
esp_err_t uart_set_rx_full_threshold(int port, int threshold) { uart_port_seen = port; assert(threshold == 1); return ESP_OK; }
esp_err_t uart_set_rx_timeout(int port, int threshold) { uart_port_seen = port; assert(threshold == 1); return ESP_OK; }
esp_err_t uart_flush_input(int port) { uart_port_seen = port; return ESP_OK; }
int uart_read_bytes(int port, void *data, size_t size, uint32_t ticks)
{
    (void)port;
    size_t available = rx_size - rx_offset;
    if (!available) { now_us += ticks * 1000; return 0; }
    if (size > available) size = available;
    if (size > rx_chunk) size = rx_chunk;
    memcpy(data, rx_bytes + rx_offset, size); rx_offset += size;
    return (int)size;
}
esp_err_t gpio_set_direction(int pin, int mode) { ++pin_calls; directions[pin] = mode; return ESP_OK; }
esp_err_t gpio_set_level(int pin, int level) { levels[pin] = level; ++pin_calls; return ESP_OK; }
esp_err_t gpio_config(const gpio_config_t *config)
{
    ++pin_calls;
    if (config_fail_pin >= 0 && (config->pin_bit_mask & (1ULL << config_fail_pin))) return ESP_FAIL;
    for (unsigned pin=0;pin<22;++pin)
        if (config->pin_bit_mask & (1ULL << pin)) directions[pin]=config->mode;
    return ESP_OK;
}
esp_err_t gpio_reset_pin(int pin)
{
    ++pin_calls;
    assert(!driver_installed && gpio_out[pin] == SIG_GPIO_OUT_IDX);
    if (pin == reset_fail_pin) return ESP_FAIL;
    directions[pin] = GPIO_MODE_INPUT; return ESP_OK;
}
void esp_rom_gpio_connect_in_signal(int pin, unsigned signal, bool invert)
{
    assert(!invert); unsigned port = (signal - 11) / 2;
    assert(port < 2); rx_route[port] = pin; selected_rx = pin;
}
void esp_rom_gpio_connect_out_signal(int pin, unsigned signal, bool invert, bool oen)
{
    assert(!invert && !oen); gpio_out[pin] = signal;
    assert(selected_rx == GPIO_MATRIX_CONST_ONE_INPUT);
    // ROM routing enables output even for SIG_GPIO_OUT_IDX.
    directions[pin] |= GPIO_MODE_OUTPUT;
}
int64_t esp_timer_get_time(void) { now_us += 100; return now_us; }
void vTaskDelay(TickType_t ticks)
{
    bool entry_high = true;
    for (unsigned pin=0;pin<2;++pin)
        entry_high = entry_high && directions[pin]==GPIO_MODE_OUTPUT && levels[pin]==1;
    if (entry_high) {
        for (unsigned pin=0;pin<2;++pin)
            assert(directions[pin]==GPIO_MODE_OUTPUT && levels[pin]==1 && gpio_out[pin]==SIG_GPIO_OUT_IDX);
        assert(CONFIG_ESC_PASSTHROUGH_BOOT_WAIT_MS && ticks<=CONFIG_ESC_PASSTHROUGH_BOOT_WAIT_MS+1);
        ++boot_waits;
    }
    now_us += ticks * 1000;
}
size_t uart_ll_get_txfifo_len(uart_dev_t *hw) { (void)hw; return 128; }
void uart_ll_write_txfifo(uart_dev_t *hw, const uint8_t *data, size_t size) { (void)hw; assert(data); fifo_sent += size; }
bool uart_ll_is_tx_idle(uart_dev_t *hw) { (void)hw; return idle; }
void uart_ll_txfifo_rst(uart_dev_t *hw) { (void)hw; }

static esc_uart_t link;
static void setup(int port)
{
    driver_installed = foreign_driver = false;
    install_ok = param_ok = delete_ok = idle = true;
    config_fail_pin = reset_fail_pin = -1;
    installs = deletes = pin_calls = fifo_sent = boot_waits = 0;
    now_us = 0;
    rx_size = rx_offset = 0; rx_chunk = 9;
    assert(esc_uart_init(&link, (int[]){0,1,-1,-1}, 2, port) == ESP_OK);
    assert(pin_calls == 0 && installs == 0 && link.selected == -1);
}
static void test_lazy_foreign_and_setup_failure(void)
{
    setup(1); driver_installed = foreign_driver = true;
    assert(esc_uart_start(&link) == ESP_ERR_INVALID_STATE && !pin_calls && !installs);
    assert(esc_uart_stop(&link) == ESP_OK && !deletes && !pin_calls && driver_installed);
    setup(1); install_ok = false;
    assert(esc_uart_start(&link) == ESP_FAIL && !pin_calls);
    assert(esc_uart_stop(&link) == ESP_OK && !deletes);
    setup(1); param_ok = false;
    assert(esc_uart_start(&link) == ESP_FAIL && driver_installed && !pin_calls);
    assert(esc_uart_stop(&link) == ESP_OK && deletes == 1 && !pin_calls);
    setup(1); config_fail_pin = 1;
    assert(esc_uart_start(&link) == ESP_FAIL && link.claimed_mask == 3);
    assert(esc_uart_stop(&link) == ESP_OK && !driver_installed && !link.claimed_mask);
    assert(!boot_waits);
}
static void test_routes_and_waveforms(void)
{
    const unsigned parked_mode = CONFIG_ESC_PASSTHROUGH_BOOT_WAIT_MS ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    const unsigned expected_waits = CONFIG_ESC_PASSTHROUGH_BOOT_WAIT_MS ? 1 : 0;
    for (int port = 0; port < 2; ++port) {
        setup(port); assert(esc_uart_start(&link) == ESP_OK);
        assert(boot_waits==0 && now_us<1000);
        assert(directions[0]==parked_mode && directions[1]==parked_mode);
        assert(esc_uart_start(&link) == ESP_OK && installs == 1 && boot_waits==0);
        esc_wire_t wire = esc_uart_wire(&link);
        wire.release(wire.ctx);
        assert(directions[0]==parked_mode && directions[1]==parked_mode);
        assert(wire.select(wire.ctx, 0) && selected_rx == 0);
        assert(boot_waits==expected_waits && now_us>=(int64_t)CONFIG_ESC_PASSTHROUGH_BOOT_WAIT_MS*1000);
        assert(directions[0]==GPIO_MODE_INPUT && directions[1]==parked_mode);
        uint8_t data[258] = {0};
        assert(wire.send(wire.ctx, data, sizeof(data)) && fifo_sent == sizeof(data));
        assert(directions[0] == GPIO_MODE_INPUT && gpio_out[0] == SIG_GPIO_OUT_IDX && selected_rx == 0);
        assert(wire.pulse_low(wire.ctx, 300));
        assert(directions[0] == GPIO_MODE_INPUT && now_us >= 300000);
        assert(wire.select(wire.ctx, 1) && selected_rx == 1);
        assert(boot_waits==expected_waits && gpio_out[0]==SIG_GPIO_OUT_IDX && directions[0]==parked_mode);
        assert(!wire.receive(wire.ctx, data, 1, 30));
        idle = false;
        assert(!wire.send(wire.ctx, data, 1));
        assert(directions[1]==GPIO_MODE_INPUT && gpio_out[1]==SIG_GPIO_OUT_IDX);
        assert(esc_uart_stop(&link) == ESP_OK && !driver_installed && !link.claimed_mask);
        assert(directions[0]==GPIO_MODE_INPUT && directions[1]==GPIO_MODE_INPUT);
        assert(rx_route[port] == GPIO_MATRIX_CONST_ONE_INPUT && uart_port_seen == (unsigned)port);
        assert(!wire.select(wire.ctx, 0));
        unsigned before = pin_calls;
        wire.release(wire.ctx); assert(pin_calls == before);
    }
}
static void test_entry_delay_consumed_by_client(void)
{
    setup(0); assert(esc_uart_start(&link)==ESP_OK);
    esc_wire_t wire=esc_uart_wire(&link);
    now_us+=2000000;
    int64_t before=now_us;
    assert(esc_uart_start(&link)==ESP_OK);
    assert(wire.select(wire.ctx,0));
    assert(now_us-before<600000);
    assert(esc_uart_stop(&link)==ESP_OK);
    setup(0); assert(esc_uart_start(&link)==ESP_OK);
    wire=esc_uart_wire(&link);
    now_us+=3000000;
    before=now_us;
    assert(wire.select(wire.ctx,0) && boot_waits==0 && now_us-before<1000);
    assert(esc_uart_stop(&link)==ESP_OK);
}
static void test_cleanup_retry(void)
{
    setup(1); assert(esc_uart_start(&link) == ESP_OK); delete_ok = false;
    assert(esc_uart_stop(&link) == ESP_FAIL && link.installed && link.claimed_mask == 3);
    assert(esc_uart_start(&link) == ESP_ERR_INVALID_STATE);
    assert(gpio_out[0] == SIG_GPIO_OUT_IDX && gpio_out[1] == SIG_GPIO_OUT_IDX);
    delete_ok = true; reset_fail_pin = 1;
    assert(esc_uart_stop(&link) == ESP_FAIL && !link.installed && link.claimed_mask == 2);
    assert(esc_uart_start(&link) == ESP_ERR_INVALID_STATE);
    unsigned before = deletes; reset_fail_pin = -1;
    assert(esc_uart_stop(&link) == ESP_OK && !link.claimed_mask && deletes == before);
    assert(esc_uart_start(&link) == ESP_OK && esc_uart_stop(&link) == ESP_OK);
}
static void test_partial_receive_diagnostics(void)
{
    setup(0); assert(esc_uart_start(&link) == ESP_OK);
    esc_wire_t wire = esc_uart_wire(&link);
    assert(wire.select(wire.ctx, 0));
    uint8_t data[9] = {0};
    memcpy(rx_bytes, "471abcdef", 9); rx_size = 5; rx_chunk = 2;
    assert(!wire.receive(wire.ctx, data, 9, 30));
    assert(wire.received_size(wire.ctx) == 5 && !memcmp(data, rx_bytes, 5));
    rx_size = 9; rx_offset = 0;
    assert(wire.receive(wire.ctx, data, 9, 30));
    assert(wire.received_size(wire.ctx) == 9 && !memcmp(data, rx_bytes, 9));
    assert(!wire.receive(wire.ctx, data, 9, 30) && wire.received_size(wire.ctx) == 0);
    assert(esc_uart_stop(&link) == ESP_OK);
}
int main(void)
{
    test_lazy_foreign_and_setup_failure();
    test_routes_and_waveforms();
    test_entry_delay_consumed_by_client();
    test_cleanup_retry();
    test_partial_receive_diagnostics();
    puts("PASS: production UART lazy setup, AM32 high entry hold/release, occupied UART isolation, routes, finite RX and cleanup retry");
    return 0;
}
