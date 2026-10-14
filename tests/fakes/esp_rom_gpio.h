// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdbool.h>
void esp_rom_gpio_connect_in_signal(int pin, unsigned signal, bool invert);
void esp_rom_gpio_connect_out_signal(int pin, unsigned signal, bool invert, bool oen_invert);
