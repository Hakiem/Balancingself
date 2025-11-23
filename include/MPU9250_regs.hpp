/**
 * @file MPU9250_regs.hpp
 * @brief Complete register map for MPU9250 9-axis IMU
 *
 * Includes:
 * - MPU9250 primary registers (gyro, accel, temp, config)
 * - AK8963 magnetometer registers (accessed via I2C bypass)
 * - Bit field definitions for configuration
 *
 * @note MPU9250 uses 7-bit I2C address: 0x68 (AD0=0) or 0x69 (AD0=1)
 * @note AK8963 magnetometer I2C address: 0x0C
 */

#pragma once

#include <cstdint>

namespace MPU9250
{

// ============================================================================
// I2C Addresses
// ============================================================================
constexpr uint8_t I2C_ADDR_AD0_LOW = 0x68; // Default address (AD0 pin LOW)
constexpr uint8_t I2C_ADDR_AD0_HIGH = 0x69; // Alternate address (AD0 pin HIGH)
constexpr uint8_t AK8963_I2C_ADDR = 0x0C; // Magnetometer address

// ============================================================================
// Primary MPU9250 Register Map
// ============================================================================

// Self-Test Registers
constexpr uint8_t SELF_TEST_X_GYRO = 0x00;
constexpr uint8_t SELF_TEST_Y_GYRO = 0x01;
constexpr uint8_t SELF_TEST_Z_GYRO = 0x02;
constexpr uint8_t SELF_TEST_X_ACCEL = 0x0D;
constexpr uint8_t SELF_TEST_Y_ACCEL = 0x0E;
constexpr uint8_t SELF_TEST_Z_ACCEL = 0x0F;

// Gyro Offset Registers (16-bit, high byte first)
constexpr uint8_t XG_OFFSET_H = 0x13;
constexpr uint8_t XG_OFFSET_L = 0x14;
constexpr uint8_t YG_OFFSET_H = 0x15;
constexpr uint8_t YG_OFFSET_L = 0x16;
constexpr uint8_t ZG_OFFSET_H = 0x17;
constexpr uint8_t ZG_OFFSET_L = 0x18;

// Sample Rate Divider
constexpr uint8_t SMPLRT_DIV
    = 0x19; // Sample Rate = Gyro Rate / (1 + SMPLRT_DIV)

// Configuration Registers
constexpr uint8_t CONFIG = 0x1A; // DLPF, FSYNC config
constexpr uint8_t GYRO_CONFIG = 0x1B; // Gyro full scale, self-test
constexpr uint8_t ACCEL_CONFIG = 0x1C; // Accel full scale, self-test
constexpr uint8_t ACCEL_CONFIG_2 = 0x1D; // Accel DLPF config

// Low Power Accelerometer ODR Control
constexpr uint8_t LP_ACCEL_ODR = 0x1E;

// Wake on Motion Threshold
constexpr uint8_t WOM_THR = 0x1F;

// FIFO Enable Register
constexpr uint8_t FIFO_EN = 0x23;

// I2C Master Control
constexpr uint8_t I2C_MST_CTRL = 0x24;
constexpr uint8_t I2C_SLV0_ADDR = 0x25;
constexpr uint8_t I2C_SLV0_REG = 0x26;
constexpr uint8_t I2C_SLV0_CTRL = 0x27;
constexpr uint8_t I2C_SLV1_ADDR = 0x28;
constexpr uint8_t I2C_SLV1_REG = 0x29;
constexpr uint8_t I2C_SLV1_CTRL = 0x2A;
constexpr uint8_t I2C_SLV2_ADDR = 0x2B;
constexpr uint8_t I2C_SLV2_REG = 0x2C;
constexpr uint8_t I2C_SLV2_CTRL = 0x2D;
constexpr uint8_t I2C_SLV3_ADDR = 0x2E;
constexpr uint8_t I2C_SLV3_REG = 0x2F;
constexpr uint8_t I2C_SLV3_CTRL = 0x30;
constexpr uint8_t I2C_SLV4_ADDR = 0x31;
constexpr uint8_t I2C_SLV4_REG = 0x32;
constexpr uint8_t I2C_SLV4_DO = 0x33;
constexpr uint8_t I2C_SLV4_CTRL = 0x34;
constexpr uint8_t I2C_SLV4_DI = 0x35;

// I2C Master Status
constexpr uint8_t I2C_MST_STATUS = 0x36;

// Interrupt Control
constexpr uint8_t INT_PIN_CFG = 0x37;
constexpr uint8_t INT_ENABLE = 0x38;
constexpr uint8_t INT_STATUS = 0x3A;

// Accelerometer Measurements (16-bit, high byte first)
constexpr uint8_t ACCEL_XOUT_H = 0x3B;
constexpr uint8_t ACCEL_XOUT_L = 0x3C;
constexpr uint8_t ACCEL_YOUT_H = 0x3D;
constexpr uint8_t ACCEL_YOUT_L = 0x3E;
constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
constexpr uint8_t ACCEL_ZOUT_L = 0x40;

// Temperature Measurement (16-bit, high byte first)
constexpr uint8_t TEMP_OUT_H = 0x41;
constexpr uint8_t TEMP_OUT_L = 0x42;

// Gyroscope Measurements (16-bit, high byte first)
constexpr uint8_t GYRO_XOUT_H = 0x43;
constexpr uint8_t GYRO_XOUT_L = 0x44;
constexpr uint8_t GYRO_YOUT_H = 0x45;
constexpr uint8_t GYRO_YOUT_L = 0x46;
constexpr uint8_t GYRO_ZOUT_H = 0x47;
constexpr uint8_t GYRO_ZOUT_L = 0x48;

// External Sensor Data (from I2C slaves)
constexpr uint8_t EXT_SENS_DATA_00 = 0x49;
constexpr uint8_t EXT_SENS_DATA_01 = 0x4A;
constexpr uint8_t EXT_SENS_DATA_02 = 0x4B;
constexpr uint8_t EXT_SENS_DATA_03 = 0x4C;
constexpr uint8_t EXT_SENS_DATA_04 = 0x4D;
constexpr uint8_t EXT_SENS_DATA_05 = 0x4E;
constexpr uint8_t EXT_SENS_DATA_06 = 0x4F;
constexpr uint8_t EXT_SENS_DATA_07 = 0x50;
constexpr uint8_t EXT_SENS_DATA_08 = 0x51;
constexpr uint8_t EXT_SENS_DATA_09 = 0x52;
constexpr uint8_t EXT_SENS_DATA_10 = 0x53;
constexpr uint8_t EXT_SENS_DATA_11 = 0x54;
constexpr uint8_t EXT_SENS_DATA_12 = 0x55;
constexpr uint8_t EXT_SENS_DATA_13 = 0x56;
constexpr uint8_t EXT_SENS_DATA_14 = 0x57;
constexpr uint8_t EXT_SENS_DATA_15 = 0x58;
constexpr uint8_t EXT_SENS_DATA_16 = 0x59;
constexpr uint8_t EXT_SENS_DATA_17 = 0x5A;
constexpr uint8_t EXT_SENS_DATA_18 = 0x5B;
constexpr uint8_t EXT_SENS_DATA_19 = 0x5C;
constexpr uint8_t EXT_SENS_DATA_20 = 0x5D;
constexpr uint8_t EXT_SENS_DATA_21 = 0x5E;
constexpr uint8_t EXT_SENS_DATA_22 = 0x5F;
constexpr uint8_t EXT_SENS_DATA_23 = 0x60;

// I2C Slave Data Out
constexpr uint8_t I2C_SLV0_DO = 0x63;
constexpr uint8_t I2C_SLV1_DO = 0x64;
constexpr uint8_t I2C_SLV2_DO = 0x65;
constexpr uint8_t I2C_SLV3_DO = 0x66;

// I2C Master Delay Control
constexpr uint8_t I2C_MST_DELAY_CTRL = 0x67;

// Signal Path Reset
constexpr uint8_t SIGNAL_PATH_RESET = 0x68;

// Accelerometer Interrupt Control
constexpr uint8_t MOT_DETECT_CTRL = 0x69;

// User Control Register
constexpr uint8_t USER_CTRL = 0x6A;

// Power Management
constexpr uint8_t PWR_MGMT_1 = 0x6B; // Device reset, sleep mode, clock source
constexpr uint8_t PWR_MGMT_2 = 0x6C; // Axis enable/disable, low power mode

// FIFO Count Registers
constexpr uint8_t FIFO_COUNTH = 0x72;
constexpr uint8_t FIFO_COUNTL = 0x73;

// FIFO Read/Write
constexpr uint8_t FIFO_R_W = 0x74;

// Device Identification
constexpr uint8_t WHO_AM_I = 0x75; // Should return 0x71 for MPU9250

// Accelerometer Offset Registers (16-bit, high byte first)
constexpr uint8_t XA_OFFSET_H = 0x77;
constexpr uint8_t XA_OFFSET_L = 0x78;
constexpr uint8_t YA_OFFSET_H = 0x7A;
constexpr uint8_t YA_OFFSET_L = 0x7B;
constexpr uint8_t ZA_OFFSET_H = 0x7D;
constexpr uint8_t ZA_OFFSET_L = 0x7E;

// ============================================================================
// AK8963 Magnetometer Register Map (accessed via I2C bypass or slave)
// ============================================================================
namespace AK8963
{
    constexpr uint8_t WIA = 0x00; // Device ID (should return 0x48)
    constexpr uint8_t INFO = 0x01; // Information
    constexpr uint8_t ST1 = 0x02; // Status 1 (data ready)
    constexpr uint8_t XOUT_L = 0x03; // X-axis measurement LSB
    constexpr uint8_t XOUT_H = 0x04; // X-axis measurement MSB
    constexpr uint8_t YOUT_L = 0x05; // Y-axis measurement LSB
    constexpr uint8_t YOUT_H = 0x06; // Y-axis measurement MSB
    constexpr uint8_t ZOUT_L = 0x07; // Z-axis measurement LSB
    constexpr uint8_t ZOUT_H = 0x08; // Z-axis measurement MSB
    constexpr uint8_t ST2 = 0x09; // Status 2 (overflow, output bit)
    constexpr uint8_t CNTL1 = 0x0A; // Control 1 (mode, output bits)
    constexpr uint8_t CNTL2 = 0x0B; // Control 2 (reset)
    constexpr uint8_t ASTC = 0x0C; // Self-test control
    constexpr uint8_t TS1 = 0x0D; // Test 1 (do not use)
    constexpr uint8_t TS2 = 0x0E; // Test 2 (do not use)
    constexpr uint8_t I2CDIS = 0x0F; // I2C disable
    constexpr uint8_t ASAX = 0x10; // X-axis sensitivity adjustment
    constexpr uint8_t ASAY = 0x11; // Y-axis sensitivity adjustment
    constexpr uint8_t ASAZ = 0x12; // Z-axis sensitivity adjustment
}

// ============================================================================
// Configuration Bit Definitions
// ============================================================================

// CONFIG register (0x1A) - DLPF Configuration
namespace CONFIG_BITS
{
    constexpr uint8_t FIFO_MODE
        = (1 << 6); // 0=overwrite old, 1=no write when full

    // DLPF_CFG - Digital Low Pass Filter bandwidth
    enum class DLPF : uint8_t {
        BW_250HZ = 0, // Gyro: 250Hz, Accel: 460Hz, Delay: 0.97ms, Fs: 8kHz
        BW_184HZ = 1, // Gyro: 184Hz, Accel: 184Hz, Delay: 2.9ms, Fs: 1kHz
        BW_92HZ = 2, // Gyro: 92Hz, Accel: 92Hz, Delay: 3.9ms, Fs: 1kHz
        BW_41HZ = 3, // Gyro: 41Hz, Accel: 41Hz, Delay: 5.9ms, Fs: 1kHz
        BW_20HZ = 4, // Gyro: 20Hz, Accel: 20Hz, Delay: 9.9ms, Fs: 1kHz
        BW_10HZ = 5, // Gyro: 10Hz, Accel: 10Hz, Delay: 17.85ms, Fs: 1kHz
        BW_5HZ = 6, // Gyro: 5Hz, Accel: 5Hz, Delay: 33.48ms, Fs: 1kHz
        BW_3600HZ = 7 // Gyro: 3600Hz, Accel: 460Hz, Delay: 0.17ms, Fs: 8kHz
    };
}

// GYRO_CONFIG register (0x1B)
namespace GYRO_CONFIG_BITS
{
    constexpr uint8_t XGYRO_ST = (1 << 7); // X gyro self-test enable
    constexpr uint8_t YGYRO_ST = (1 << 6); // Y gyro self-test enable
    constexpr uint8_t ZGYRO_ST = (1 << 5); // Z gyro self-test enable

    // Gyro Full Scale Range
    enum class FS_SEL : uint8_t {
        DPS_250 = (0 << 3), // ±250 °/s, 131 LSB/°/s
        DPS_500 = (1 << 3), // ±500 °/s, 65.5 LSB/°/s
        DPS_1000 = (2 << 3), // ±1000 °/s, 32.8 LSB/°/s
        DPS_2000 = (3 << 3) // ±2000 °/s, 16.4 LSB/°/s
    };

    // Fchoice for gyro - disables DLPF if set
    constexpr uint8_t FCHOICE_B_MASK = 0x03;
}

// ACCEL_CONFIG register (0x1C)
namespace ACCEL_CONFIG_BITS
{
    constexpr uint8_t AX_ST = (1 << 7); // X accel self-test enable
    constexpr uint8_t AY_ST = (1 << 6); // Y accel self-test enable
    constexpr uint8_t AZ_ST = (1 << 5); // Z accel self-test enable

    // Accelerometer Full Scale Range
    enum class AFS_SEL : uint8_t {
        G_2 = (0 << 3), // ±2g, 16384 LSB/g
        G_4 = (1 << 3), // ±4g, 8192 LSB/g
        G_8 = (2 << 3), // ±8g, 4096 LSB/g
        G_16 = (3 << 3) // ±16g, 2048 LSB/g
    };
}

// ACCEL_CONFIG_2 register (0x1D)
namespace ACCEL_CONFIG_2_BITS
{
    // Accel DLPF Configuration
    enum class A_DLPF_CFG : uint8_t {
        BW_460HZ = 0, // 460Hz bandwidth, 1.94ms delay, Fs=1kHz
        BW_184HZ = 1, // 184Hz bandwidth, 5.80ms delay
        BW_92HZ = 2, // 92Hz bandwidth, 7.80ms delay
        BW_41HZ = 3, // 41Hz bandwidth, 11.80ms delay
        BW_20HZ = 4, // 20Hz bandwidth, 19.80ms delay
        BW_10HZ = 5, // 10Hz bandwidth, 35.70ms delay
        BW_5HZ = 6, // 5Hz bandwidth, 66.96ms delay
        BW_460HZ_NOLPF = 7 // 460Hz, 1.94ms, no LPF
    };

    constexpr uint8_t ACCEL_FCHOICE_B = (1 << 3); // Disable accel DLPF
}

// FIFO_EN register (0x23)
namespace FIFO_EN_BITS
{
    constexpr uint8_t TEMP_OUT = (1 << 7); // Write TEMP_OUT to FIFO
    constexpr uint8_t GYRO_XOUT = (1 << 6); // Write GYRO_XOUT to FIFO
    constexpr uint8_t GYRO_YOUT = (1 << 5); // Write GYRO_YOUT to FIFO
    constexpr uint8_t GYRO_ZOUT = (1 << 4); // Write GYRO_ZOUT to FIFO
    constexpr uint8_t ACCEL = (1 << 3); // Write ACCEL_XOUT/Y/Z to FIFO
    constexpr uint8_t SLV_2 = (1 << 2); // Write EXT_SENS_DATA from SLV_2
    constexpr uint8_t SLV_1 = (1 << 1); // Write EXT_SENS_DATA from SLV_1
    constexpr uint8_t SLV_0 = (1 << 0); // Write EXT_SENS_DATA from SLV_0
}

// I2C_MST_CTRL register (0x24)
namespace I2C_MST_CTRL_BITS
{
    constexpr uint8_t MULT_MST_EN = (1 << 7); // Multi-master enable
    constexpr uint8_t WAIT_FOR_ES = (1 << 6); // Delay data ready until ext data
    constexpr uint8_t SLV_3_FIFO_EN = (1 << 5); // Write SLV_3 data to FIFO
    constexpr uint8_t I2C_MST_P_NSR = (1 << 4); // Stop between reads

    // I2C Master Clock Speed
    enum class I2C_MST_CLK : uint8_t {
        CLK_348KHZ = 0, // 348 kHz
        CLK_333KHZ = 1, // 333 kHz
        CLK_320KHZ = 2, // 320 kHz
        CLK_308KHZ = 3, // 308 kHz
        CLK_296KHZ = 4, // 296 kHz
        CLK_286KHZ = 5, // 286 kHz
        CLK_276KHZ = 6, // 276 kHz
        CLK_267KHZ = 7, // 267 kHz
        CLK_258KHZ = 8, // 258 kHz
        CLK_500KHZ = 9, // 500 kHz
        CLK_471KHZ = 10, // 471 kHz
        CLK_444KHZ = 11, // 444 kHz
        CLK_421KHZ = 12, // 421 kHz
        CLK_400KHZ = 13, // 400 kHz
        CLK_381KHZ = 14, // 381 kHz
        CLK_364KHZ = 15 // 364 kHz
    };
}

// INT_PIN_CFG register (0x37)
namespace INT_PIN_CFG_BITS
{
    constexpr uint8_t ACTL = (1 << 7); // INT pin active low
    constexpr uint8_t OPEN = (1 << 6); // INT pin open drain
    constexpr uint8_t LATCH_INT_EN = (1 << 5); // Latch INT until cleared
    constexpr uint8_t INT_ANYRD_2CLEAR = (1 << 4); // Clear INT on any read
    constexpr uint8_t ACTL_FSYNC = (1 << 3); // FSYNC pin active low
    constexpr uint8_t FSYNC_INT_MODE_EN = (1 << 2); // Enable FSYNC interrupt
    constexpr uint8_t BYPASS_EN = (1 << 1); // I2C bypass enable (for mag)
}

// INT_ENABLE register (0x38)
namespace INT_ENABLE_BITS
{
    constexpr uint8_t WOM_EN = (1 << 6); // Wake on motion interrupt
    constexpr uint8_t FIFO_OVERFLOW_EN = (1 << 4); // FIFO overflow interrupt
    constexpr uint8_t FSYNC_INT_EN = (1 << 3); // FSYNC interrupt enable
    constexpr uint8_t RAW_RDY_EN = (1 << 0); // Raw data ready interrupt
}

// INT_STATUS register (0x3A)
namespace INT_STATUS_BITS
{
    constexpr uint8_t WOM_INT = (1 << 6); // Wake on motion interrupt
    constexpr uint8_t FIFO_OVERFLOW_INT = (1 << 4); // FIFO overflow interrupt
    constexpr uint8_t FSYNC_INT = (1 << 3); // FSYNC interrupt occurred
    constexpr uint8_t RAW_DATA_RDY_INT = (1 << 0); // Raw data ready interrupt
}

// USER_CTRL register (0x6A)
namespace USER_CTRL_BITS
{
    constexpr uint8_t FIFO_EN = (1 << 6); // Enable FIFO operation
    constexpr uint8_t I2C_MST_EN = (1 << 5); // Enable I2C Master mode
    constexpr uint8_t I2C_IF_DIS = (1 << 4); // Disable I2C slave (use SPI only)
    constexpr uint8_t FIFO_RST = (1 << 2); // Reset FIFO
    constexpr uint8_t I2C_MST_RST = (1 << 1); // Reset I2C Master
    constexpr uint8_t SIG_COND_RST = (1 << 0); // Reset signal paths
}

// PWR_MGMT_1 register (0x6B)
namespace PWR_MGMT_1_BITS
{
    constexpr uint8_t H_RESET = (1 << 7); // Device reset
    constexpr uint8_t SLEEP = (1 << 6); // Sleep mode enable
    constexpr uint8_t CYCLE = (1 << 5); // Cycle between sleep and waking
    constexpr uint8_t GYRO_STANDBY = (1 << 4); // Gyro drive disabled
    constexpr uint8_t PD_PTAT = (1 << 3); // Power down temp sensor

    // Clock Source Select
    enum class CLKSEL : uint8_t {
        INTERNAL_20MHZ = 0, // Internal 20MHz oscillator
        AUTO_PLL = 1, // Auto select best clock (recommended)
        STOP_CLOCK = 7 // Stop clock, keep timing generator reset
    };
}

// PWR_MGMT_2 register (0x6C)
namespace PWR_MGMT_2_BITS
{
    constexpr uint8_t DISABLE_XA = (1 << 5); // Disable X accelerometer
    constexpr uint8_t DISABLE_YA = (1 << 4); // Disable Y accelerometer
    constexpr uint8_t DISABLE_ZA = (1 << 3); // Disable Z accelerometer
    constexpr uint8_t DISABLE_XG = (1 << 2); // Disable X gyro
    constexpr uint8_t DISABLE_YG = (1 << 1); // Disable Y gyro
    constexpr uint8_t DISABLE_ZG = (1 << 0); // Disable Z gyro
}

// ============================================================================
// AK8963 Magnetometer Configuration Bits
// ============================================================================
namespace AK8963_BITS
{
    // ST1 register (0x02) - Status 1
    constexpr uint8_t DRDY = (1 << 0); // Data ready
    constexpr uint8_t DOR = (1 << 1); // Data overrun

    // ST2 register (0x09) - Status 2
    constexpr uint8_t BITM = (1 << 4); // Output bit setting (14/16-bit)
    constexpr uint8_t HOFL = (1 << 3); // Magnetic sensor overflow

    // CNTL1 register (0x0A) - Control 1
    enum class MODE : uint8_t {
        POWER_DOWN = 0x00, // Power-down mode
        SINGLE_MEASURE = 0x01, // Single measurement mode
        CONT_MEASURE_1 = 0x02, // Continuous measurement mode 1 (8Hz)
        CONT_MEASURE_2 = 0x06, // Continuous measurement mode 2 (100Hz)
        EXT_TRIGGER = 0x04, // External trigger measurement
        SELF_TEST = 0x08, // Self-test mode
        FUSE_ROM_ACCESS = 0x0F // Fuse ROM access mode
    };

    enum class OUTPUT_BITS : uint8_t {
        BIT_14 = (0 << 4), // 14-bit output (0.6 µT/LSB)
        BIT_16 = (1 << 4) // 16-bit output (0.15 µT/LSB)
    };

    // CNTL2 register (0x0B) - Control 2
    constexpr uint8_t SRST = (1 << 0); // Soft reset
}

// ============================================================================
// Sensitivity Scale Factors
// ============================================================================
namespace SENSITIVITY
{
    // Gyroscope sensitivity (LSB/°/s)
    constexpr float GYRO_250DPS = 131.0f;
    constexpr float GYRO_500DPS = 65.5f;
    constexpr float GYRO_1000DPS = 32.8f;
    constexpr float GYRO_2000DPS = 16.4f;

    // Accelerometer sensitivity (LSB/g)
    constexpr float ACCEL_2G = 16384.0f;
    constexpr float ACCEL_4G = 8192.0f;
    constexpr float ACCEL_8G = 4096.0f;
    constexpr float ACCEL_16G = 2048.0f;

    // Magnetometer sensitivity (µT/LSB)
    constexpr float MAG_14BIT = 0.6f; // 14-bit mode
    constexpr float MAG_16BIT = 0.15f; // 16-bit mode

    // Temperature sensitivity
    constexpr float TEMP_SENSITIVITY = 333.87f; // LSB/°C
    constexpr float TEMP_OFFSET = 21.0f; // °C at 0 LSB
}

// ============================================================================
// Expected Device IDs
// ============================================================================
constexpr uint8_t MPU9250_WHO_AM_I_VALUE = 0x71;
constexpr uint8_t MPU9255_WHO_AM_I_VALUE = 0x73; // MPU9255 variant
constexpr uint8_t AK8963_WHO_AM_I_VALUE = 0x48;

// ============================================================================
// Timing Constants
// ============================================================================
constexpr uint32_t RESET_DELAY_MS = 100; // Delay after reset
constexpr uint32_t MAG_INIT_DELAY_MS = 10; // Delay for magnetometer init
constexpr uint32_t FIFO_MAX_SIZE = 512; // FIFO buffer size in bytes

} // namespace MPU9250
