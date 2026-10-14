// SPDX-License-Identifier: GPL-3.0-or-later
#include "usb_host.h"

void app_main(void)
{
    const int pins[PT_MAX_CHANNELS] = {CONFIG_ESC1_GPIO, CONFIG_ESC2_GPIO,
                                     CONFIG_ESC3_GPIO, CONFIG_ESC4_GPIO};
    esc_passthrough_config_t config = usb_config(pins);
    esc_passthrough_t *session = NULL;
    ESP_ERROR_CHECK(esc_passthrough_create(&config, &session));
    bool was_connected = false;
    for (;;) usb_pump(session, &was_connected);
}
