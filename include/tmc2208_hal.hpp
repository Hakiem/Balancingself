#pragma once

#include "board.h"
#include "tmc2208_uart.h"

#include <cstddef>

namespace tmc2208_hal
{

using LogFunc = void (*)(const char* tag, const uint8_t* payload, size_t len);

tmc2208::Driver make_driver(UART_HandleTypeDef* uart, uint8_t slave_address = 0);
void set_logger(LogFunc func);

}
