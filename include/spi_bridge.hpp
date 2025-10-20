#pragma once

#include "logger.hpp"
#ifdef UNIT_TEST
#include "hal_stubs.hpp"
#else
#include "stm32f3xx_hal.h"
#endif
#include <cstddef>
#include <cstdint>

/**
 * @brief Thin wrapper around STM32 HAL SPI to simplify CS handling and logging.
 *
 * Typical usage:
 * @code
 * SPI_Bridge spi(&hspi1, GPIOA, GPIO_PIN_4);
 * auto guard = spi.scopedSelect();
 * uint8_t tx[5] = { 0x80 | addr, data3, data2, data1, data0 };
 * spi.transmit(tx, sizeof(tx));
 * @endcode
 */
class SPI_Bridge
{
public:
    /**
     * @brief Construct a new SPI bridge.
     *
     * @param spi       Pointer to the HAL SPI handle.
     * @param cs_port   GPIO port for the chip-select signal.
     * @param cs_pin    GPIO pin for the chip-select signal.
     * @param timeout_ms Timeout applied to HAL transfers (default 10 ms).
     */
    SPI_Bridge(
        SPI_HandleTypeDef* spi,
        GPIO_TypeDef* cs_port,
        uint16_t cs_pin,
        uint32_t timeout_ms = 10)
        : spi_(spi)
        , cs_port_(cs_port)
        , cs_pin_(cs_pin)
        , timeout_ms_(timeout_ms)
    {
        // Ensure CS is de-asserted on construction so the peripheral is idle.
        deselect();
    }

    /**
     * @brief RAII helper that asserts CS on construction and releases on scope exit.
     */
    class ScopedSelect
    {
    public:
        explicit ScopedSelect(SPI_Bridge& bridge)
            : bridge_(&bridge)
        {
            bridge_->select();
        }

        ScopedSelect(const ScopedSelect&) = delete;
        ScopedSelect& operator=(const ScopedSelect&) = delete;

        ScopedSelect(ScopedSelect&& other) noexcept
            : bridge_(other.bridge_)
            , active_(other.active_)
        {
            other.bridge_ = nullptr;
            other.active_ = false;
        }

        ScopedSelect& operator=(ScopedSelect&& other) noexcept
        {
            if (this != &other) {
                release();
                bridge_ = other.bridge_;
                active_ = other.active_;
                other.bridge_ = nullptr;
                other.active_ = false;
            }
            return *this;
        }

        ~ScopedSelect() { release(); }

    private:
        void release()
        {
            if (bridge_ && active_) {
                bridge_->deselect();
                active_ = false;
            }
        }

        SPI_Bridge* bridge_ = nullptr;
        bool active_ = true;
    };

    /**
     * @brief Acquire chip-select for the duration of the returned guard.
     */
    [[nodiscard]] ScopedSelect scopedSelect() { return ScopedSelect(*this); }

    /**
     * @brief Manually drive chip-select low.
     */
    void select()
    {
        if (cs_port_) {
            HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
            __NOP();
        }
    }

    /**
     * @brief Manually release chip-select (drive high).
     */
    void deselect()
    {
        if (cs_port_) {
            HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
            __NOP();
        }
    }

    /**
     * @brief Update the SPI transfer timeout.
     */
    void setTimeout(uint32_t timeout_ms) { timeout_ms_ = timeout_ms; }

    /**
     * @brief Current SPI transfer timeout (in ms).
     */
    uint32_t timeout() const { return timeout_ms_; }

    /**
     * @brief Full-duplex transfer.
     *
     * @param tx   Bytes to transmit (may be nullptr to send zeroes).
     * @param rx   Buffer to store received bytes (may be nullptr to discard).
     * @param len  Number of bytes to exchange.
     * @return true on success, false otherwise.
     */
    bool transfer(const uint8_t* tx, uint8_t* rx, size_t len)
    {
        if (!spi_ || len == 0)
            return true;

        HAL_StatusTypeDef status = HAL_OK;

        if (tx && rx) {
            status = HAL_SPI_TransmitReceive(
                spi_, const_cast<uint8_t*>(tx), rx, len, timeout_ms_);
        } else if (tx) {
            status = HAL_SPI_Transmit(spi_, const_cast<uint8_t*>(tx), len, timeout_ms_);
        } else if (rx) {
            status = HAL_SPI_Receive(spi_, rx, len, timeout_ms_);
        } else {
            return true;
        }

        if (status != HAL_OK) {
#ifndef UNIT_TEST
            logsys::printf("[SPI] transfer failed (status=%d)\r\n", status);
#endif
            return false;
        }
        return true;
    }

    /**
     * @brief Convenience wrapper around @ref transfer for TX-only operations.
     */
    bool transmit(const uint8_t* tx, size_t len) { return transfer(tx, nullptr, len); }

    /**
     * @brief Convenience wrapper around @ref transfer for RX-only operations.
     */
    bool receive(uint8_t* rx, size_t len) { return transfer(nullptr, rx, len); }

private:
    SPI_HandleTypeDef* spi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;
    uint32_t timeout_ms_;
};
