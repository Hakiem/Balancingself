#pragma once
#ifdef UNIT_TEST
#include "hal_stubs.hpp"
#else
#include "stm32f3xx_hal.h"
#endif
#include <cstdarg>
#include <cstddef>
#include <cstdint>

namespace logsys
{

using TransmitFn = HAL_StatusTypeDef (*)(
    UART_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);

// Initialize UART used for logging
void init(UART_HandleTypeDef* huart);

// Override the transmitter (useful for unit testing)
void set_transmitter(TransmitFn fn);

// Print a formatted string (print-style)
void printf(const char* format, ...);

// Hex dump (Useful for raw UART traffic)
void dump(const char* tag, const uint8_t* data, size_t len);

} // namespace logsys
