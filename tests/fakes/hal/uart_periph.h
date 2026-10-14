// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#define SOC_UART_HP_NUM 2
#define SOC_UART_PERIPH_SIGNAL_RX 1
#define SOC_UART_PERIPH_SIGNAL_TX 0
#define UART_PERIPH_SIGNAL(port, type) ((unsigned)(10 + 2 * (port) + (type)))
