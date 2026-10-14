// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
#define configMAX_PRIORITIES 25
#define configNUMBER_OF_CORES 1
