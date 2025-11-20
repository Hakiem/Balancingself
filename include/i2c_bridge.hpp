/**
 * @file i2c_bridge.hpp
 * @brief I2C communication bridge for sensor access
 *
 * Provides a clean interface for reading/writing registers on I2C devices
 * like the MPU9250, abstracting away HAL-specific calls.
 */

#pragma once

#include "stm32f3xx_hal.h"
#include <cstdint>

/**
 * @class I2CBridge
 * @brief Wrapper for I2C communication with sensors
 *
 * Handles single/multiple byte reads and writes with timeout management.
 * Typical usage:
 *   I2CBridge i2c(&hi2c1);
 *   uint8_t whoami = i2c.readRegister(0x68, 0x75);
 *   i2c.writeRegister(0x68, 0x6B, 0x00);  // Wake up MPU9250
 */
class I2CBridge
{
public:
    /**
     * @brief Construct I2C bridge with HAL handle
     * @param hi2c Pointer to HAL I2C handle (e.g., &hi2c1)
     * @param timeout_ms Default timeout in milliseconds (default: 100ms)
     */
    explicit I2CBridge(I2C_HandleTypeDef* hi2c, uint32_t timeout_ms = 100)
        : hi2c_(hi2c)
        , timeout_ms_(timeout_ms)
        , last_status_(HAL_OK)
    {
    }

    /**
     * @brief Write a single byte to a register
     * @param device_addr 7-bit I2C device address (will be shifted internally)
     * @param reg_addr Register address to write to
     * @param data Data byte to write
     * @return true if successful, false otherwise
     */
    bool writeRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
    {
        uint8_t buffer[2] = { reg_addr, data };
        last_status_ = HAL_I2C_Master_Transmit(
            hi2c_, device_addr << 1, buffer, 2, timeout_ms_);
        return last_status_ == HAL_OK;
    }

    /**
     * @brief Write multiple bytes starting at a register address
     * @param device_addr 7-bit I2C device address
     * @param reg_addr Starting register address
     * @param data Pointer to data buffer
     * @param length Number of bytes to write
     * @return true if successful, false otherwise
     */
    bool writeRegisters(uint8_t device_addr, uint8_t reg_addr,
        const uint8_t* data, uint16_t length)
    {
        // Option 1: Write register address first, then data (standard approach)
        last_status_ = HAL_I2C_Mem_Write(hi2c_, device_addr << 1, reg_addr,
            I2C_MEMADD_SIZE_8BIT, const_cast<uint8_t*>(data), length,
            timeout_ms_);
        return last_status_ == HAL_OK;
    }

    /**
     * @brief Read a single byte from a register
     * @param device_addr 7-bit I2C device address
     * @param reg_addr Register address to read from
     * @param data Reference to store the read byte
     * @return true if successful, false otherwise
     */
    bool readRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t& data)
    {
        last_status_ = HAL_I2C_Mem_Read(hi2c_, device_addr << 1, reg_addr,
            I2C_MEMADD_SIZE_8BIT, &data, 1, timeout_ms_);
        return last_status_ == HAL_OK;
    }

    /**
     * @brief Read a single byte from a register (return value version)
     * @param device_addr 7-bit I2C device address
     * @param reg_addr Register address to read from
     * @return Read byte value (0 on error - check lastStatus())
     */
    uint8_t readRegister(uint8_t device_addr, uint8_t reg_addr)
    {
        uint8_t data = 0;
        readRegister(device_addr, reg_addr, data);
        return data;
    }

    /**
     * @brief Read multiple bytes starting at a register address
     * @param device_addr 7-bit I2C device address
     * @param reg_addr Starting register address
     * @param data Pointer to buffer to store read data
     * @param length Number of bytes to read
     * @return true if successful, false otherwise
     */
    bool readRegisters(
        uint8_t device_addr, uint8_t reg_addr, uint8_t* data, uint16_t length)
    {
        last_status_ = HAL_I2C_Mem_Read(hi2c_, device_addr << 1, reg_addr,
            I2C_MEMADD_SIZE_8BIT, data, length, timeout_ms_);
        return last_status_ == HAL_OK;
    }

    /**
     * @brief Modify bits in a register (read-modify-write)
     * @param device_addr 7-bit I2C device address
     * @param reg_addr Register address
     * @param mask Bit mask (bits to modify)
     * @param value New value for masked bits
     * @return true if successful, false otherwise
     *
     * Example: Set bit 5, clear bit 3
     *   modifyRegister(0x68, 0x1A, 0x28, 0x20);  // mask=0b00101000,
     * value=0b00100000
     */
    bool modifyRegister(
        uint8_t device_addr, uint8_t reg_addr, uint8_t mask, uint8_t value)
    {
        uint8_t reg_value;
        if (!readRegister(device_addr, reg_addr, reg_value)) {
            return false;
        }
        reg_value = (reg_value & ~mask) | (value & mask);
        return writeRegister(device_addr, reg_addr, reg_value);
    }

    /**
     * @brief Set specific bits in a register
     * @param device_addr 7-bit I2C device address
     * @param reg_addr Register address
     * @param bits Bits to set (1 = set, 0 = leave unchanged)
     * @return true if successful, false otherwise
     */
    bool setBits(uint8_t device_addr, uint8_t reg_addr, uint8_t bits)
    {
        uint8_t reg_value;
        if (!readRegister(device_addr, reg_addr, reg_value)) {
            return false;
        }
        reg_value |= bits;
        return writeRegister(device_addr, reg_addr, reg_value);
    }

    /**
     * @brief Clear specific bits in a register
     * @param device_addr 7-bit I2C device address
     * @param reg_addr Register address
     * @param bits Bits to clear (1 = clear, 0 = leave unchanged)
     * @return true if successful, false otherwise
     */
    bool clearBits(uint8_t device_addr, uint8_t reg_addr, uint8_t bits)
    {
        uint8_t reg_value;
        if (!readRegister(device_addr, reg_addr, reg_value)) {
            return false;
        }
        reg_value &= ~bits;
        return writeRegister(device_addr, reg_addr, reg_value);
    }

    /**
     * @brief Read a 16-bit value from two consecutive registers (MSB first)
     * @param device_addr 7-bit I2C device address
     * @param reg_addr_high High byte register address
     * @param value Reference to store the 16-bit value
     * @return true if successful, false otherwise
     */
    bool readRegister16(
        uint8_t device_addr, uint8_t reg_addr_high, int16_t& value)
    {
        uint8_t buffer[2];
        if (!readRegisters(device_addr, reg_addr_high, buffer, 2)) {
            return false;
        }
        value = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
        return true;
    }

    /**
     * @brief Check if a device is present on the I2C bus
     * @param device_addr 7-bit I2C device address
     * @param trials Number of trials (default: 3)
     * @return true if device responds, false otherwise
     */
    bool isDeviceReady(uint8_t device_addr, uint8_t trials = 3)
    {
        last_status_ = HAL_I2C_IsDeviceReady(
            hi2c_, device_addr << 1, trials, timeout_ms_);
        return last_status_ == HAL_OK;
    }

    /**
     * @brief Get the last HAL status code
     * @return HAL_StatusTypeDef from last operation
     */
    HAL_StatusTypeDef lastStatus() const { return last_status_; }

    /**
     * @brief Set timeout for I2C operations
     * @param timeout_ms Timeout in milliseconds
     */
    void setTimeout(uint32_t timeout_ms) { timeout_ms_ = timeout_ms; }

    /**
     * @brief Get current timeout setting
     * @return Timeout in milliseconds
     */
    uint32_t getTimeout() const { return timeout_ms_; }

    /**
     * @brief Scan I2C bus for devices
     * @param devices Array to store found device addresses (7-bit)
     * @param max_devices Maximum number of devices to find
     * @return Number of devices found
     */
    uint8_t scanBus(uint8_t* devices, uint8_t max_devices)
    {
        uint8_t count = 0;
        for (uint8_t addr = 0x03; addr < 0x78 && count < max_devices; addr++) {
            if (isDeviceReady(addr, 1)) {
                devices[count++] = addr;
            }
        }
        return count;
    }

private:
    I2C_HandleTypeDef* hi2c_; ///< HAL I2C handle
    uint32_t timeout_ms_; ///< I2C operation timeout
    HAL_StatusTypeDef last_status_; ///< Last operation status
};
