// SPDX-License-Identifier: GPL-3.0-or-later
#include "usb_host.h"
#include "control.hpp"

static bool motor_usb_output(void *ctx, const uint8_t *data, size_t size)
{
    bool ok = usb_output(nullptr, data, size);
    if (!ok) static_cast<MotorOwner*>(ctx)->output_failed();
    return ok;
}

extern "C" void app_main(void)
{
    if (CONFIG_EXAMPLE_DSHOT_GPIO < 0) return;
    const int pins[PT_MAX_CHANNELS] = {CONFIG_EXAMPLE_DSHOT_GPIO, -1, -1, -1};
    esc_passthrough_config_t config = usb_config(pins);
    dshot::Config dshot_config = MotorOwner::config();
    dshot::Driver driver(dshot_config);
    dshot::Link link(driver, static_cast<gpio_num_t>(pins[0]));
    MotorOwner motors(driver, link, esp_timer_get_time);
    example_control::Context control{motors};
    config.lifecycle = motors.callbacks();
    config.application_command = example_control::command;
    config.application_ctx = &control;
    config.output = motor_usb_output;
    config.output_ctx = &motors;
    esc_passthrough_t *session = nullptr;
    ESP_ERROR_CHECK(esc_passthrough_create(&config, &session));
    control.session = session;
    motors.attach_session(session);
    bool was_connected = false;
    for (;;) {
        if (usb_pump(session, &was_connected)) motors.transport_lost();
        motors.poll();
    }
}
