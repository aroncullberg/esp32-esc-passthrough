// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "esp_err.h"
#include <stdint.h>
#define GPIO_IS_VALID_OUTPUT_GPIO(pin) ((pin) >= 0 && (pin) <= 21)
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_INPUT_OUTPUT,
       GPIO_PULLUP_ENABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE };
typedef struct { uint64_t pin_bit_mask; int mode, pull_up_en, pull_down_en, intr_type; } gpio_config_t;
esp_err_t gpio_set_direction(int pin, int mode);
esp_err_t gpio_set_level(int pin, int level);
esp_err_t gpio_config(const gpio_config_t *config);
esp_err_t gpio_reset_pin(int pin);
typedef int gpio_num_t;
#define GPIO_NUM_NC (-1)
