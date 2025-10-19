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

        // Enable transmitter mode
        HAL_HalfDuplex_EnableTransmitter(uart_);

        // Small delay to let hardware settle
        for (volatile int i = 0; i < 100; i++) { } // ~1µs delay

        // Send data
        HAL_StatusTypeDef status = HAL_UART_Transmit(
            uart_, const_cast<uint8_t*>(data), len, HAL_MAX_DELAY);

        if (status != HAL_OK) {
            logsys::dump("TX_FAIL", data, len);
            return false;
        }

        // Wait for transmission to fully complete
        while (__HAL_UART_GET_FLAG(uart_, UART_FLAG_TC) == RESET) {
            // Wait for transmission complete flag
        }

        // Additional small delay to ensure line settles
        for (volatile int i = 0; i < 200; i++) { } // ~2µs delay

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

        // Enable receiver mode
        HAL_HalfDuplex_EnableReceiver(uart_);

        // Small delay to let hardware settle into RX mode
        for (volatile int i = 0; i < 100; i++) { } // ~1µs delay

        // Clear any pending RX flags before starting
        __HAL_UART_CLEAR_FLAG(uart_,
            UART_FLAG_RXNE | UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_PE);

        // Receive data
        HAL_StatusTypeDef res = HAL_UART_Receive(uart_, data, len, timeout_ms);
        size_t received = (res == HAL_OK) ? len : 0u;

        // Enhanced logging for debugging
        if (received == len) {
            logsys::dump("RX", data, received);
        } else if (received > 0) {
            logsys::dump("RX_PARTIAL", data, received);
        } else {
            // Check why we failed
            if (res == HAL_TIMEOUT) {
                logsys::dump("RX_TIMEOUT", data, 0);
            } else {
                logsys::dump("RX_ERROR", data, 0);
            }
        }

        return received;
    }

private:
    UART_HandleTypeDef* uart_;
};