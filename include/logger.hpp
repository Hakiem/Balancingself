#pragma once
#include "stm32f3xx_hal.h"
#include <cstdarg>
#include <cstddef>
#include <cstdint>

namespace logsys
{

// Initialize UART used for logging
void init(UART_HandleTypeDef* huart);

// Print a formatted string (print-style)
void printf(const char* format, ...);

// Hex dump (Useful for raw UART traffic)
void dump(const char* tag, const uint8_t* data, size_t len);

} // namespace logsys