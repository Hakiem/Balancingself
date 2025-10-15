#pragma once
#include "logger.hpp"
#include "stm32f3xx_hal.h"
#include <cstddef>
#include <cstdint>

class UART_Bridge
{
public:
    explicit UART_Bridge(UART_HandleTypeDef* uart)
        : uart_(uart)
    {
    }

    // Send bytes (Half-Duplex)
    bool transmit(const uint8_t* data, size_t len)
    {
        if (!uart_)
            return false;
        HAL_HalfDuplex_EnableTransmitter(uart_);
        if (HAL_UART_Transmit(
                uart_, const_cast<uint8_t*>(data), len, HAL_MAX_DELAY)
            != HAL_OK)
            return false;
        logsys::dump("TX", data, len);
        return true;
    }

    // Receive bytes (Half-Duplex)
    size_t receive(uint8_t* data, size_t len, uint32_t timeout_us)
    {
        if (!uart_)
            return 0;
        uint32_t timeout_ms = (timeout_us + 999u) / 1000u;
        if (timeout_ms == 0u)
            timeout_ms = 1u;
        HAL_HalfDuplex_EnableReceiver(uart_);
        HAL_StatusTypeDef res = HAL_UART_Receive(uart_, data, len, timeout_ms);
        size_t received = (res == HAL_OK) ? len : 0u;
        logsys::dump(received == len ? "RX" : "RX_SHORT", data, received);
        return received;
    }

private:
    UART_HandleTypeDef* uart_;
};