/**
 * @file mpu9250.hpp
 * @brief MPU9250 9-axis IMU driver with DMP support (Header-only)
 *
 * High-level driver for MPU9250 that uses the Digital Motion Processor (DMP)
 * to perform sensor fusion on-chip, providing quaternion output that can be
 * converted to roll, pitch, and yaw angles.
 *
 * Key features:
 * - DMP-based sensor fusion (quaternion output)
 * - Automatic gyro calibration
 * - Roll, pitch, yaw angle calculation
 * - Temperature reading
 * - Magnetometer support (AK8963)
 * - Configurable sample rates and ranges
 * - FIFO management
 * - Interrupt support
 */

#pragma once

#include "MPU9250_regs.hpp"
#include "dmp_firmware.hpp"
#include "i2c_bridge.hpp"
#include "logger.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>

// Define M_PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @class MPU9250
 * @brief Complete MPU9250 driver with DMP support
 *
 * Usage example:
 *   MPU9250 imu(&i2c_bridge, MPU9250::I2C_ADDR_AD0_LOW);
 *   imu.initialize();
 *   imu.initializeDMP();
 *
 *   // In main loop:
 *   if (imu.dataReady()) {
 *       imu.update();
 *       float roll = imu.getRoll();
 *       float pitch = imu.getPitch();
 *       float yaw = imu.getYaw();
 *   }
 */
class MPU9250Driver
{
public:
    /**
     * @brief Quaternion structure for orientation
     */
    struct Quaternion {
        float w, x, y, z;

        Quaternion()
            : w(1.0f)
            , x(0.0f)
            , y(0.0f)
            , z(0.0f)
        {
        }
        Quaternion(float w_, float x_, float y_, float z_)
            : w(w_)
            , x(x_)
            , y(y_)
            , z(z_)
        {
        }

        // Normalize quaternion
        void normalize()
        {
            float mag = sqrtf(w * w + x * x + y * y + z * z);
            if (mag > 0.0f) {
                w /= mag;
                x /= mag;
                y /= mag;
                z /= mag;
            }
        }
    };

    /**
     * @brief Euler angles structure (degrees)
     */
    struct EulerAngles {
        float roll; // Rotation around X axis
        float pitch; // Rotation around Y axis
        float yaw; // Rotation around Z axis

        EulerAngles()
            : roll(0.0f)
            , pitch(0.0f)
            , yaw(0.0f)
        {
        }
    };

    /**
     * @brief 3D vector for gyro, accel, mag data
     */
    struct Vector3 {
        float x, y, z;

        Vector3()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
        {
        }
        Vector3(float x_, float y_, float z_)
            : x(x_)
            , y(y_)
            , z(z_)
        {
        }
    };

    /**
     * @brief Configuration structure
     */
    struct Config {
        MPU9250::GYRO_CONFIG_BITS::FS_SEL gyro_fsr;
        MPU9250::ACCEL_CONFIG_BITS::AFS_SEL accel_fsr;
        MPU9250::CONFIG_BITS::DLPF dlpf;
        uint16_t sample_rate_hz;
        bool enable_magnetometer;
        bool enable_interrupts;
        uint16_t dmp_features;

        Config()
            : gyro_fsr(MPU9250::GYRO_CONFIG_BITS::FS_SEL::DPS_2000)
            , accel_fsr(MPU9250::ACCEL_CONFIG_BITS::AFS_SEL::G_2)
            , dlpf(MPU9250::CONFIG_BITS::DLPF::BW_20HZ)
            , sample_rate_hz(MPU9250::DMP::BALANCE_SAMPLE_RATE)
            , enable_magnetometer(false)
            , enable_interrupts(true)
            , dmp_features(MPU9250::DMP::FEATURES_BALANCE)
        {
        }
    };

public:
    /**
     * @brief Constructor
     * @param i2c Pointer to I2C bridge
     * @param address 7-bit I2C address (default: 0x68)
     */
    explicit MPU9250Driver(
        I2CBridge* i2c, uint8_t address = MPU9250::I2C_ADDR_AD0_LOW)
        : i2c_(i2c)
        , address_(address)
        , dmp_enabled_(false)
        , mag_enabled_(false)
        , gyro_scale_(MPU9250::SENSITIVITY::GYRO_2000DPS)
        , accel_scale_(MPU9250::SENSITIVITY::ACCEL_2G)
    {
    }

    // ========================================================================
    // Initialization
    // ========================================================================

    /**
     * @brief Initialize MPU9250 (basic configuration)
     * @param config Configuration structure
     * @return true if successful
     */
    bool initialize(const Config& config = Config());

    /**
     * @brief Initialize DMP (Digital Motion Processor)
     * @return true if successful
     */
    bool initializeDMP();

    /**
     * @brief Complete initialization with DMP and calibration
     * Performs: basic init, DMP init, gyro calibration, interrupt setup
     * @param config Configuration structure (optional)
     * @param calibration_samples Number of samples for gyro calibration
     * (default: 200)
     * @return true if successful
     */
    bool initializeWithDMP(
        const Config& config = Config(), uint16_t calibration_samples = 200);

    /**
     * @brief Reset device to default state
     * @return true if successful
     */
    bool reset();

    /**
     * @brief Check if device is connected
     * @return true if WHO_AM_I matches expected value
     */
    bool testConnection();

    // ========================================================================
    // DMP Control
    // ========================================================================

    /**
     * @brief Enable DMP
     * @return true if successful
     */
    bool enableDMP();

    /**
     * @brief Disable DMP
     * @return true if successful
     */
    bool disableDMP();

    /**
     * @brief Check if DMP is enabled
     */
    bool isDMPEnabled() const { return dmp_enabled_; }

    /**
     * @brief Set DMP features
     * @param features Feature flags (see MPU9250::DMP::FEATURE_*)
     * @return true if successful
     */
    bool setDMPFeatures(uint16_t features);

    /**
     * @brief Set DMP sample rate
     * @param rate_hz Sample rate in Hz (4-200)
     * @return true if successful
     */
    bool setDMPSampleRate(uint16_t rate_hz);

    // ========================================================================
    // Data Acquisition
    // ========================================================================

    /**
     * @brief Check if new data is available
     * @return true if data ready
     */
    bool dataReady();

    /**
     * @brief Update all sensor data from DMP FIFO
     * @return true if successful
     */
    bool update();

    /**
     * @brief Read raw gyroscope data
     * @param gyro Output vector (deg/s)
     * @return true if successful
     */
    bool readGyro(Vector3& gyro);

    /**
     * @brief Read raw accelerometer data
     * @param accel Output vector (g)
     * @return true if successful
     */
    bool readAccel(Vector3& accel);

    /**
     * @brief Read magnetometer data
     * @param mag Output vector (µT)
     * @return true if successful
     */
    bool readMag(Vector3& mag);

    /**
     * @brief Read temperature
     * @return Temperature in °C
     */
    float readTemperature();

    // ========================================================================
    // Orientation (Primary Interface for Balancing)
    // ========================================================================

    /**
     * @brief Get roll angle (rotation around X axis)
     * @return Roll in degrees (-180 to +180)
     */
    float getRoll() const { return euler_.roll; }

    /**
     * @brief Get pitch angle (rotation around Y axis)
     * @return Pitch in degrees (-90 to +90)
     */
    float getPitch() const { return euler_.pitch; }

    /**
     * @brief Get yaw angle (rotation around Z axis)
     * @return Yaw in degrees (0 to 360)
     */
    float getYaw() const { return euler_.yaw; }

    /**
     * @brief Get all Euler angles
     * @return EulerAngles structure
     */
    const EulerAngles& getEulerAngles() const { return euler_; }

    /**
     * @brief Get quaternion
     * @return Quaternion structure
     */
    const Quaternion& getQuaternion() const { return quat_; }

    /**
     * @brief Get gyroscope data (deg/s)
     */
    const Vector3& getGyro() const { return gyro_; }

    /**
     * @brief Get accelerometer data (g)
     */
    const Vector3& getAccel() const { return accel_; }

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Set gyroscope full scale range
     * @param fsr Full scale range
     * @return true if successful
     */
    bool setGyroFSR(MPU9250::GYRO_CONFIG_BITS::FS_SEL fsr);

    /**
     * @brief Set accelerometer full scale range
     * @param fsr Full scale range
     * @return true if successful
     */
    bool setAccelFSR(MPU9250::ACCEL_CONFIG_BITS::AFS_SEL fsr);

    /**
     * @brief Set digital low-pass filter
     * @param dlpf DLPF setting
     * @return true if successful
     */
    bool setDLPF(MPU9250::CONFIG_BITS::DLPF dlpf);

    /**
     * @brief Set sample rate divider
     * @param rate_hz Desired sample rate (Hz)
     * @return true if successful
     */
    bool setSampleRate(uint16_t rate_hz);

    // ========================================================================
    // FIFO Control
    // ========================================================================

    /**
     * @brief Enable FIFO
     * @return true if successful
     */
    bool enableFIFO();

    /**
     * @brief Disable FIFO
     * @return true if successful
     */
    bool disableFIFO();

    /**
     * @brief Reset FIFO
     * @return true if successful
     */
    bool resetFIFO();

    /**
     * @brief Get FIFO count
     * @return Number of bytes in FIFO
     */
    uint16_t getFIFOCount();

    /**
     * @brief Read data from FIFO
     * @param data Output buffer
     * @param length Number of bytes to read
     * @return true if successful
     */
    bool readFIFO(uint8_t* data, uint16_t length);

    // ========================================================================
    // Interrupt Configuration
    // ========================================================================

    /**
     * @brief Enable data ready interrupt
     * @return true if successful
     */
    bool enableInterrupt();

    /**
     * @brief Disable data ready interrupt
     * @return true if successful
     */
    bool disableInterrupt();

    /**
     * @brief Configure interrupt pin
     * @param active_low true for active low, false for active high
     * @param open_drain true for open drain, false for push-pull
     * @param latch true to latch until read, false for 50µs pulse
     * @param clear_on_read true to clear interrupt on any register read
     * @return true if successful
     */
    bool configureInterruptPin(bool active_low = false, bool open_drain = false,
        bool latch = true, bool clear_on_read = true);

    // ========================================================================
    // Magnetometer (AK8963)
    // ========================================================================

    /**
     * @brief Initialize magnetometer
     * @return true if successful
     */
    bool initializeMagnetometer();

    /**
     * @brief Enable magnetometer
     * @return true if successful
     */
    bool enableMagnetometer();

    /**
     * @brief Disable magnetometer
     * @return true if successful
     */
    bool disableMagnetometer();

    // ========================================================================
    // Calibration
    // ========================================================================

    /**
     * @brief Calibrate gyroscope (device must be stationary)
     * @param samples Number of samples to average (default: 1000)
     * @return true if successful
     */
    bool calibrateGyro(uint16_t samples = 1000);

    /**
     * @brief Calibrate accelerometer
     * @param samples Number of samples to average (default: 1000)
     * @return true if successful
     */
    bool calibrateAccel(uint16_t samples = 1000);

    /**
     * @brief Load gyro bias offsets
     * @param offset_x X axis offset
     * @param offset_y Y axis offset
     * @param offset_z Z axis offset
     * @return true if successful
     */
    bool setGyroBias(int16_t offset_x, int16_t offset_y, int16_t offset_z);

    // ========================================================================
    // Utility
    // ========================================================================

    /**
     * @brief Get device address
     */
    uint8_t getAddress() const { return address_; }

    /**
     * @brief Get last error status
     */
    const char* getLastError() const { return last_error_; }

private:
    // I2C communication
    I2CBridge* i2c_;
    uint8_t address_;

    // DMP state
    bool dmp_enabled_;
    bool mag_enabled_;

    // Sensor data
    Quaternion quat_;
    EulerAngles euler_;
    Vector3 gyro_;
    Vector3 accel_;
    Vector3 mag_;
    float temperature_;

    // Scale factors
    float gyro_scale_;
    float accel_scale_;
    float mag_scale_x_;
    float mag_scale_y_;
    float mag_scale_z_;

    // Error tracking
    const char* last_error_;

    // DMP firmware (will be loaded from external array)
    const uint8_t* dmp_firmware_;
    uint16_t dmp_firmware_size_;

    // Helper functions
    bool writeDMPMemory(uint16_t mem_addr, const uint8_t* data, uint16_t length)
    {
        uint8_t bank = (mem_addr >> 8) & 0xFF;
        uint8_t offset = mem_addr & 0xFF;

        if (!setMemoryBank(bank)) {
            return false;
        }

        if (!setMemoryStartAddress(offset)) {
            return false;
        }

        return i2c_->writeRegisters(address_, MPU9250::MEM_R_W, data, length);
    }

    bool readDMPMemory(uint16_t mem_addr, uint8_t* data, uint16_t length)
    {
        uint8_t bank = (mem_addr >> 8) & 0xFF;
        uint8_t offset = mem_addr & 0xFF;

        if (!setMemoryBank(bank)) {
            return false;
        }

        if (!setMemoryStartAddress(offset)) {
            return false;
        }

        return i2c_->readRegisters(address_, MPU9250::MEM_R_W, data, length);
    }

    bool setMemoryBank(uint8_t bank)
    {
        return i2c_->writeRegister(address_, MPU9250::BANK_SEL, bank);
    }

    bool setMemoryStartAddress(uint8_t addr)
    {
        return i2c_->writeRegister(address_, MPU9250::MEM_START_ADDR, addr);
    }

    bool setDMPStartAddress(uint16_t start_addr)
    {
        uint8_t data[2] = { static_cast<uint8_t>((start_addr >> 8) & 0xFF),
            static_cast<uint8_t>(start_addr & 0xFF) };
        return i2c_->writeRegisters(
            address_, MPU9250::DMP_CFG_1, data, sizeof(data));
    }

    bool loadDMPFirmware(const uint8_t* firmware, uint16_t size)
    {
        // Load firmware in chunks to avoid I2C timeout
        constexpr uint16_t CHUNK_SIZE = 16; // Write 16 bytes at a time
        uint16_t bytes_written = 0;

        logsys::printf(
            "[MPU9250] Loading firmware in %d-byte chunks...\r\n", CHUNK_SIZE);

        while (bytes_written < size) {
            uint16_t chunk_size = (size - bytes_written > CHUNK_SIZE)
                ? CHUNK_SIZE
                : (size - bytes_written);

            if (!writeDMPMemory(
                    bytes_written, &firmware[bytes_written], chunk_size)) {
                logsys::printf(
                    "[MPU9250] Firmware write failed at offset %d\r\n",
                    bytes_written);
                return false;
            }

            bytes_written += chunk_size;

            // Print progress every 256 bytes
            if (bytes_written % 256 == 0 || bytes_written == size) {
                logsys::printf("[MPU9250] Progress: %d/%d bytes (%.1f%%)\r\n",
                    bytes_written, size, (bytes_written * 100.0f) / size);
            }
        }

        // Verify firmware by reading back first few bytes
        uint8_t verify[16];
        if (readDMPMemory(0, verify, 16)) {
            bool match = true;
            for (int i = 0; i < 16; i++) {
                if (verify[i] != firmware[i]) {
                    match = false;
                    break;
                }
            }

            if (match) {
                logsys::printf("[MPU9250] Firmware verification passed\r\n");
            } else {
                logsys::printf(
                    "[MPU9250] WARNING: Firmware verification failed\r\n");
                return false;
            }
        }

        if (!setDMPStartAddress(MPU9250::DMP::START_ADDRESS)) {
            logsys::printf(
                "[MPU9250] ERROR: Failed to set DMP start address\r\n");
            return false;
        }
        logsys::printf("[MPU9250] DMP start address set to 0x%04X\r\n",
            MPU9250::DMP::START_ADDRESS);

        return true;
    }

    void quaternionToEuler(const Quaternion& q, EulerAngles& e)
    {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
        float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        e.roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (q.w * q.y - q.z * q.x);
        if (fabsf(sinp) >= 1.0f) {
            e.pitch = copysignf(90.0f, sinp); // Use ±90° if out of range
        } else {
            e.pitch = asinf(sinp) * 180.0f / M_PI;
        }

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
        float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        e.yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;

        // Normalize yaw to 0-360
        if (e.yaw < 0.0f) {
            e.yaw += 360.0f;
        }
    }

    bool processDMPPacket(const uint8_t* packet, uint16_t length)
    {
        // Parse DMP packet format
        if (length < MPU9250::DMP::BALANCE_PACKET_SIZE) {
            return false;
        }

        // Skip header (2 bytes)
        const uint8_t* data = packet + 2;

        // Extract 6-axis quaternion (Q30 format)
        int32_t quat_w = (static_cast<int32_t>(data[0]) << 24)
            | (static_cast<int32_t>(data[1]) << 16)
            | (static_cast<int32_t>(data[2]) << 8) | data[3];
        int32_t quat_x = (static_cast<int32_t>(data[4]) << 24)
            | (static_cast<int32_t>(data[5]) << 16)
            | (static_cast<int32_t>(data[6]) << 8) | data[7];
        int32_t quat_y = (static_cast<int32_t>(data[8]) << 24)
            | (static_cast<int32_t>(data[9]) << 16)
            | (static_cast<int32_t>(data[10]) << 8) | data[11];

        // Convert to float
        quat_.w = MPU9250::DMP::quatToFloat(quat_w);
        quat_.x = MPU9250::DMP::quatToFloat(quat_x);
        quat_.y = MPU9250::DMP::quatToFloat(quat_y);

        // Calculate z component
        float sum_sq
            = quat_.w * quat_.w + quat_.x * quat_.x + quat_.y * quat_.y;
        if (sum_sq < 1.0f) {
            quat_.z = sqrtf(1.0f - sum_sq);
        } else {
            quat_.z = 0.0f;
            quat_.normalize();
        }

        // Extract gyro data (offset 12, 6 bytes)
        const uint8_t* gyro_data = data + 12;
        int16_t raw_gx
            = (static_cast<int16_t>(gyro_data[0]) << 8) | gyro_data[1];
        int16_t raw_gy
            = (static_cast<int16_t>(gyro_data[2]) << 8) | gyro_data[3];
        int16_t raw_gz
            = (static_cast<int16_t>(gyro_data[4]) << 8) | gyro_data[5];

        gyro_.x = static_cast<float>(raw_gx) / gyro_scale_;
        gyro_.y = static_cast<float>(raw_gy) / gyro_scale_;
        gyro_.z = static_cast<float>(raw_gz) / gyro_scale_;

        return true;
    }

    // Magnetometer helpers
    bool writeMagRegister(uint8_t reg, uint8_t value)
    {
        return i2c_->writeRegister(MPU9250::AK8963_I2C_ADDR, reg, value);
    }

    bool readMagRegister(uint8_t reg, uint8_t& value)
    {
        return i2c_->readRegister(MPU9250::AK8963_I2C_ADDR, reg, value);
    }

    bool readMagRegisters(uint8_t reg, uint8_t* data, uint8_t length)
    {
        return i2c_->readRegisters(MPU9250::AK8963_I2C_ADDR, reg, data, length);
    }
};

// ============================================================================
// INLINE IMPLEMENTATIONS
// ============================================================================

inline bool MPU9250Driver::initialize(const Config& config)
{
    last_error_ = nullptr;

    if (!testConnection()) {
        last_error_ = "Device not found";
        return false;
    }

    logsys::printf("[MPU9250] Device found (WHO_AM_I = 0x71)\r\n");

    if (!reset()) {
        last_error_ = "Reset failed";
        return false;
    }

    // Wake up from sleep
    if (!i2c_->writeRegister(address_, MPU9250::PWR_MGMT_1,
            static_cast<uint8_t>(MPU9250::PWR_MGMT_1_BITS::CLKSEL::AUTO_PLL))) {
        last_error_ = "Wake up failed";
        return false;
    }
    HAL_Delay(10);

    // Disable I2C master mode initially
    i2c_->clearBits(
        address_, MPU9250::USER_CTRL, MPU9250::USER_CTRL_BITS::I2C_MST_EN);

    if (!setGyroFSR(config.gyro_fsr)) {
        last_error_ = "Gyro FSR config failed";
        return false;
    }

    if (!setAccelFSR(config.accel_fsr)) {
        last_error_ = "Accel FSR config failed";
        return false;
    }

    if (!setDLPF(config.dlpf)) {
        last_error_ = "DLPF config failed";
        return false;
    }

    if (!setSampleRate(config.sample_rate_hz)) {
        last_error_ = "Sample rate config failed";
        return false;
    }

    if (config.enable_interrupts) {
        if (!configureInterruptPin()) {
            last_error_ = "Interrupt config failed";
            return false;
        }
    }

    if (config.enable_magnetometer) {
        mag_enabled_ = initializeMagnetometer();
        if (!mag_enabled_) {
            logsys::printf("[MPU9250] Warning: Magnetometer init failed\r\n");
        }
    }

    logsys::printf("[MPU9250] Basic initialization complete\r\n");
    return true;
}

inline bool MPU9250Driver::initializeDMP()
{
    logsys::printf("[MPU9250] Initializing DMP...\r\n");

    // Reset FIFO and DMP
    i2c_->writeRegister(address_, MPU9250::USER_CTRL,
        MPU9250::USER_CTRL_BITS::FIFO_RST
            | MPU9250::USER_CTRL_BITS::SIG_COND_RST);
    HAL_Delay(10);

    logsys::printf(
        "[MPU9250] Loading DMP firmware (%d bytes)...\r\n", DMP::FIRMWARE_SIZE);

    // Load DMP firmware into MPU9250 memory banks
    if (!loadDMPFirmware(DMP::FIRMWARE, DMP::FIRMWARE_SIZE)) {
        last_error_ = "DMP firmware load failed";
        logsys::printf("[MPU9250] ERROR: Firmware loading failed!\r\n");
        return false;
    }
    logsys::printf("[MPU9250] Firmware loaded successfully\r\n");

    // Configure INT pin: active-low, push-pull, pulse on data ready
    logsys::printf(
        "[MPU9250] Configuring INT pin (active-low, latched)...\r\n");
    // Configure INT pin as active-low, push-pull, latched until read and
    // cleared on any register read. This helps avoid missed short pulses.
    if (!configureInterruptPin(true, false, true, true)) {
        last_error_ = "INT pin config failed";
        return false;
    }

    // Set DMP features BEFORE enabling DMP
    if (!setDMPFeatures(MPU9250::DMP::FEATURES_BALANCE)) {
        last_error_ = "DMP features config failed";
        return false;
    }

    // Set DMP sample rate BEFORE enabling DMP
    if (!setDMPSampleRate(MPU9250::DMP::BALANCE_SAMPLE_RATE)) {
        last_error_ = "DMP sample rate config failed";
        return false;
    }

    // Clear FIFO_EN register (0x23) - required for DMP mode
    // In DMP mode, the DMP controls what goes into FIFO, not this register
    if (!i2c_->writeRegister(address_, MPU9250::FIFO_EN, 0x00)) {
        last_error_ = "FIFO_EN clear failed";
        return false;
    }

    // Enable DMP and FIFO together
    logsys::printf("[MPU9250] Enabling DMP and FIFO...\r\n");
    uint8_t user_ctrl
        = MPU9250::USER_CTRL_BITS::DMP_EN | MPU9250::USER_CTRL_BITS::FIFO_EN;
    if (!i2c_->writeRegister(address_, MPU9250::USER_CTRL, user_ctrl)) {
        last_error_ = "DMP/FIFO enable failed";
        return false;
    }

    // Enable DMP interrupt (data ready from DMP/FIFO)
    logsys::printf("[MPU9250] Enabling DMP interrupt...\r\n");
    if (!enableInterrupt()) {
        last_error_ = "Interrupt enable failed";
        return false;
    }

    // Reset FIFO AFTER enabling everything (critical for DMP to start writing)
    logsys::printf("[MPU9250] Resetting FIFO to start DMP operation...\r\n");
    if (!resetFIFO()) {
        last_error_ = "FIFO reset failed";
        return false;
    }
    HAL_Delay(50); // Give DMP time to start populating FIFO

    dmp_enabled_ = true;
    logsys::printf("[MPU9250] DMP initialized successfully\r\n");
    return true;
}

inline bool MPU9250Driver::initializeWithDMP(
    const Config& config, uint16_t calibration_samples)
{
    logsys::printf(
        "[MPU9250] Starting complete initialization with DMP...\r\n");

    // Step 1: Basic initialization
    if (!initialize(config)) {
        logsys::printf("[MPU9250] ERROR: Basic initialization failed!\r\n");
        return false;
    }
    logsys::printf("[MPU9250] Basic init OK\r\n");

    // Step 2: DMP initialization
    if (!initializeDMP()) {
        logsys::printf("[MPU9250] ERROR: DMP initialization failed!\r\n");
        logsys::printf(
            "[MPU9250] Note: DMP firmware loading not yet implemented\r\n");
        return false;
    }
    logsys::printf("[MPU9250] DMP enabled - sensor fusion active\r\n");

    // Step 3: Gyro calibration (sensor must be stationary)
    logsys::printf("[MPU9250] Calibrating gyro (keep sensor still)...\r\n");
    if (!calibrateGyro(calibration_samples)) {
        logsys::printf("[MPU9250] WARNING: Gyro calibration failed\r\n");
    } else {
        logsys::printf("[MPU9250] Gyro calibration complete\r\n");
    }

    logsys::printf(
        "[MPU9250] Initialization complete! Ready for operation.\r\n");
    logsys::printf(
        "[MPU9250] INT pin configured for data-ready interrupts\r\n");
    return true;
}

inline bool MPU9250Driver::reset()
{
    if (!i2c_->writeRegister(
            address_, MPU9250::PWR_MGMT_1, MPU9250::PWR_MGMT_1_BITS::H_RESET)) {
        return false;
    }

    HAL_Delay(MPU9250::RESET_DELAY_MS);

    uint8_t pwr_mgmt;
    if (!i2c_->readRegister(address_, MPU9250::PWR_MGMT_1, pwr_mgmt)) {
        return false;
    }

    return (pwr_mgmt & MPU9250::PWR_MGMT_1_BITS::H_RESET) == 0;
}

inline bool MPU9250Driver::testConnection()
{
    uint8_t who_am_i;
    if (!i2c_->readRegister(address_, MPU9250::WHO_AM_I, who_am_i)) {
        return false;
    }

    return (who_am_i == MPU9250::MPU9250_WHO_AM_I_VALUE
        || who_am_i == MPU9250::MPU9255_WHO_AM_I_VALUE);
}

inline bool MPU9250Driver::enableDMP()
{
    // Enable DMP and FIFO
    return i2c_->setBits(address_, MPU9250::USER_CTRL,
        MPU9250::USER_CTRL_BITS::DMP_EN | MPU9250::USER_CTRL_BITS::FIFO_EN);
}

inline bool MPU9250Driver::disableDMP()
{
    dmp_enabled_ = false;
    return i2c_->clearBits(address_, MPU9250::USER_CTRL,
        MPU9250::USER_CTRL_BITS::DMP_EN | MPU9250::USER_CTRL_BITS::FIFO_EN);
}

inline bool MPU9250Driver::setDMPFeatures(uint16_t features)
{
    // Based on SparkFun library's dmp_enable_feature()
    // This function writes to multiple DMP memory locations to configure
    // features

    // Enable 6-axis quaternion (CFG_8 = 2718)
    if (features & MPU9250::DMP::FEATURE_6X_LP_QUAT) {
        uint8_t quat_data[4]
            = { 0x20, 0x28, 0x30, 0x38 }; // DINA20, DINA28, DINA30, DINA38
        if (!writeDMPMemory(2718, quat_data, 4)) {
            logsys::printf("[MPU9250] Failed to enable 6x quaternion\r\n");
            return false;
        }
    }

    // Enable gyro calibration (CFG_MOTION_BIAS = 1208)
    if (features & MPU9250::DMP::FEATURE_GYRO_CAL) {
        uint8_t gyro_cal_data[9]
            = { 0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d };
        if (!writeDMPMemory(1208, gyro_cal_data, 9)) {
            logsys::printf("[MPU9250] Failed to enable gyro calibration\r\n");
            return false;
        }
    }

    // Write integration scale factor (D_0_104 = 0x0668)
    // GYRO_SF = 46850825 for 2000 dps gyro range
    uint8_t gyro_sf[4] = { 0x02, 0xCB, 0x47, 0xA9 }; // 46850825 in hex
    if (!writeDMPMemory(MPU9250::DMP::D_0_104, gyro_sf, 4)) {
        logsys::printf("[MPU9250] Failed to write gyro scale factor\r\n");
        return false;
    }

    // Enable or disable TAP feature (CFG_20 = 2224)
    // TAP must be enabled for FIFO to work correctly (known MPU9250 issue)
    if (features & MPU9250::DMP::FEATURE_TAP) {
        uint8_t tap_enable = 0xF8;
        if (!writeDMPMemory(2224, &tap_enable, 1)) {
            logsys::printf("[MPU9250] Failed to enable TAP\r\n");
            return false;
        }
    } else {
        uint8_t tap_disable = 0xD8;
        if (!writeDMPMemory(2224, &tap_disable, 1)) {
            logsys::printf("[MPU9250] Failed to disable TAP\r\n");
            return false;
        }
    }

    // Configure what sensor data gets sent to FIFO (CFG_15 = 2727)
    // All 0xA3 means no raw sensor data (we only want quaternion)
    uint8_t fifo_sensors[10]
        = { 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3 };
    if (!writeDMPMemory(2727, fifo_sensors, 10)) {
        logsys::printf("[MPU9250] Failed to configure FIFO sensors\r\n");
        return false;
    }

    // Configure gesture data to FIFO (CFG_27 = 2742)
    // When TAP is enabled, this should be 0x20 to send gesture data
    uint8_t gesture_cfg = (features & MPU9250::DMP::FEATURE_TAP) ? 0x20 : 0xD8;
    if (!writeDMPMemory(2742, &gesture_cfg, 1)) {
        logsys::printf("[MPU9250] Failed to configure gesture FIFO\r\n");
        return false;
    }

    // Reset FIFO after all feature configuration (matches SparkFun sequence)
    if (!resetFIFO()) {
        logsys::printf("[MPU9250] Failed to reset FIFO after features\r\n");
        return false;
    }

    logsys::printf("[MPU9250] DMP features set: 0x%04X\r\n", features);
    return true;
}

inline bool MPU9250Driver::setDMPSampleRate(uint16_t rate_hz)
{
    if (rate_hz < MPU9250::DMP::MIN_SAMPLE_RATE
        || rate_hz > MPU9250::DMP::MAX_SAMPLE_RATE) {
        return false;
    }

    // Calculate FIFO rate divider: DMP runs at 200Hz internally
    // divider = (200 / desired_rate) - 1
    uint16_t divider = (200 / rate_hz) - 1;

    // Write to DMP memory at CFG_FIFO_RATE (D_0_22 = 22)
    uint8_t rate_data[2] = { static_cast<uint8_t>((divider >> 8) & 0xFF),
        static_cast<uint8_t>(divider & 0xFF) };

    if (!writeDMPMemory(MPU9250::DMP::D_0_22, rate_data, 2)) {
        logsys::printf("[MPU9250] Failed to set DMP sample rate divider\r\n");
        return false;
    }

    // CRITICAL: Also write to CFG_6 (address 2753) - required for FIFO to work!
    // This data sequence is from SparkFun's dmp_set_fifo_rate function
    uint8_t cfg6_data[12] = { 0xfe, 0xf2, 0xab, 0xc4, 0xaa, 0xf1, 0xdf, 0xdf,
        0xbb, 0xaf, 0xdf, 0xdf };
    if (!writeDMPMemory(2753, cfg6_data, 12)) {
        logsys::printf("[MPU9250] Failed to write CFG_6\r\n");
        return false;
    }

    logsys::printf("[MPU9250] DMP sample rate set: %dHz (divider=%d)\r\n",
        rate_hz, divider);
    return true;
}

inline bool MPU9250Driver::dataReady()
{
    uint8_t status;
    if (!i2c_->readRegister(address_, MPU9250::INT_STATUS, status)) {
        return false;
    }

    uint8_t mask = dmp_enabled_ ? MPU9250::INT_STATUS_BITS::DMP_INT
                                : MPU9250::INT_STATUS_BITS::RAW_DATA_RDY_INT;
    return (status & mask) != 0;
}

inline bool MPU9250Driver::update()
{
    if (!dmp_enabled_) {
        readGyro(gyro_);
        readAccel(accel_);
        return true;
    }

    uint16_t fifo_count = getFIFOCount();

    if (fifo_count == 0) {
        return false;
    }

    if (fifo_count >= MPU9250::FIFO_MAX_SIZE) {
        logsys::printf("[MPU9250] FIFO overflow! Resetting...\r\n");
        resetFIFO();
        return false;
    }

    uint8_t packet[MPU9250::DMP::BALANCE_PACKET_SIZE];

    if (fifo_count >= MPU9250::DMP::BALANCE_PACKET_SIZE) {
        if (!readFIFO(packet, MPU9250::DMP::BALANCE_PACKET_SIZE)) {
            return false;
        }

        if (!processDMPPacket(packet, MPU9250::DMP::BALANCE_PACKET_SIZE)) {
            return false;
        }

        quaternionToEuler(quat_, euler_);

        return true;
    }

    return false;
}

inline bool MPU9250Driver::readGyro(Vector3& gyro)
{
    uint8_t data[6];
    if (!i2c_->readRegisters(address_, MPU9250::GYRO_XOUT_H, data, 6)) {
        return false;
    }

    int16_t raw_x = (static_cast<int16_t>(data[0]) << 8) | data[1];
    int16_t raw_y = (static_cast<int16_t>(data[2]) << 8) | data[3];
    int16_t raw_z = (static_cast<int16_t>(data[4]) << 8) | data[5];

    gyro.x = static_cast<float>(raw_x) / gyro_scale_;
    gyro.y = static_cast<float>(raw_y) / gyro_scale_;
    gyro.z = static_cast<float>(raw_z) / gyro_scale_;

    gyro_ = gyro;
    return true;
}

inline bool MPU9250Driver::readAccel(Vector3& accel)
{
    uint8_t data[6];
    if (!i2c_->readRegisters(address_, MPU9250::ACCEL_XOUT_H, data, 6)) {
        return false;
    }

    int16_t raw_x = (static_cast<int16_t>(data[0]) << 8) | data[1];
    int16_t raw_y = (static_cast<int16_t>(data[2]) << 8) | data[3];
    int16_t raw_z = (static_cast<int16_t>(data[4]) << 8) | data[5];

    accel.x = static_cast<float>(raw_x) / accel_scale_;
    accel.y = static_cast<float>(raw_y) / accel_scale_;
    accel.z = static_cast<float>(raw_z) / accel_scale_;

    accel_ = accel;
    return true;
}

inline bool MPU9250Driver::readMag(Vector3& mag)
{
    if (!mag_enabled_) {
        return false;
    }

    uint8_t status;
    if (!readMagRegister(MPU9250::AK8963::ST1, status)) {
        return false;
    }

    if ((status & MPU9250::AK8963_BITS::DRDY) == 0) {
        return false;
    }

    uint8_t data[7];
    if (!readMagRegisters(MPU9250::AK8963::XOUT_L, data, 7)) {
        return false;
    }

    if (data[6] & MPU9250::AK8963_BITS::HOFL) {
        return false;
    }

    int16_t raw_x = (static_cast<int16_t>(data[1]) << 8) | data[0];
    int16_t raw_y = (static_cast<int16_t>(data[3]) << 8) | data[2];
    int16_t raw_z = (static_cast<int16_t>(data[5]) << 8) | data[4];

    mag.x = static_cast<float>(raw_x) * mag_scale_x_;
    mag.y = static_cast<float>(raw_y) * mag_scale_y_;
    mag.z = static_cast<float>(raw_z) * mag_scale_z_;

    mag_ = mag;
    return true;
}

inline float MPU9250Driver::readTemperature()
{
    uint8_t data[2];
    if (!i2c_->readRegisters(address_, MPU9250::TEMP_OUT_H, data, 2)) {
        return 0.0f;
    }

    int16_t raw_temp = (static_cast<int16_t>(data[0]) << 8) | data[1];

    temperature_ = (static_cast<float>(raw_temp)
                       / MPU9250::SENSITIVITY::TEMP_SENSITIVITY)
        + MPU9250::SENSITIVITY::TEMP_OFFSET;

    return temperature_;
}

inline bool MPU9250Driver::setGyroFSR(MPU9250::GYRO_CONFIG_BITS::FS_SEL fsr)
{
    if (!i2c_->writeRegister(
            address_, MPU9250::GYRO_CONFIG, static_cast<uint8_t>(fsr))) {
        return false;
    }

    switch (fsr) {
    case MPU9250::GYRO_CONFIG_BITS::FS_SEL::DPS_250:
        gyro_scale_ = MPU9250::SENSITIVITY::GYRO_250DPS;
        break;
    case MPU9250::GYRO_CONFIG_BITS::FS_SEL::DPS_500:
        gyro_scale_ = MPU9250::SENSITIVITY::GYRO_500DPS;
        break;
    case MPU9250::GYRO_CONFIG_BITS::FS_SEL::DPS_1000:
        gyro_scale_ = MPU9250::SENSITIVITY::GYRO_1000DPS;
        break;
    case MPU9250::GYRO_CONFIG_BITS::FS_SEL::DPS_2000:
        gyro_scale_ = MPU9250::SENSITIVITY::GYRO_2000DPS;
        break;
    }

    return true;
}

inline bool MPU9250Driver::setAccelFSR(MPU9250::ACCEL_CONFIG_BITS::AFS_SEL fsr)
{
    if (!i2c_->writeRegister(
            address_, MPU9250::ACCEL_CONFIG, static_cast<uint8_t>(fsr))) {
        return false;
    }

    switch (fsr) {
    case MPU9250::ACCEL_CONFIG_BITS::AFS_SEL::G_2:
        accel_scale_ = MPU9250::SENSITIVITY::ACCEL_2G;
        break;
    case MPU9250::ACCEL_CONFIG_BITS::AFS_SEL::G_4:
        accel_scale_ = MPU9250::SENSITIVITY::ACCEL_4G;
        break;
    case MPU9250::ACCEL_CONFIG_BITS::AFS_SEL::G_8:
        accel_scale_ = MPU9250::SENSITIVITY::ACCEL_8G;
        break;
    case MPU9250::ACCEL_CONFIG_BITS::AFS_SEL::G_16:
        accel_scale_ = MPU9250::SENSITIVITY::ACCEL_16G;
        break;
    }

    return true;
}

inline bool MPU9250Driver::setDLPF(MPU9250::CONFIG_BITS::DLPF dlpf)
{
    return i2c_->modifyRegister(
        address_, MPU9250::CONFIG, 0x07, static_cast<uint8_t>(dlpf));
}

inline bool MPU9250Driver::setSampleRate(uint16_t rate_hz)
{
    if (rate_hz == 0 || rate_hz > 1000) {
        return false;
    }

    uint8_t divider = (1000 / rate_hz) - 1;
    return i2c_->writeRegister(address_, MPU9250::SMPLRT_DIV, divider);
}

inline bool MPU9250Driver::enableFIFO()
{
    return i2c_->setBits(
        address_, MPU9250::USER_CTRL, MPU9250::USER_CTRL_BITS::FIFO_EN);
}

inline bool MPU9250Driver::disableFIFO()
{
    return i2c_->clearBits(
        address_, MPU9250::USER_CTRL, MPU9250::USER_CTRL_BITS::FIFO_EN);
}

inline bool MPU9250Driver::resetFIFO()
{
    if (!i2c_->setBits(
            address_, MPU9250::USER_CTRL, MPU9250::USER_CTRL_BITS::FIFO_RST)) {
        return false;
    }
    HAL_Delay(1);
    return true;
}

inline uint16_t MPU9250Driver::getFIFOCount()
{
    uint8_t data[2];
    if (!i2c_->readRegisters(address_, MPU9250::FIFO_COUNTH, data, 2)) {
        return 0;
    }

    return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

inline bool MPU9250Driver::readFIFO(uint8_t* data, uint16_t length)
{
    return i2c_->readRegisters(address_, MPU9250::FIFO_R_W, data, length);
}

inline bool MPU9250Driver::enableInterrupt()
{
    // For DMP mode: ONLY enable DMP_INT_EN (bit 1 = 0x02)
    // Other interrupt bits (RAW_RDY, FIFO_OVERFLOW) interfere with DMP
    // operation
    uint8_t bits = MPU9250::INT_ENABLE_BITS::DMP_INT_EN; // 0x02 only

    bool result = i2c_->writeRegister(address_, MPU9250::INT_ENABLE, bits);

    if (result) {
        logsys::printf("[MPU9250] INT_ENABLE = 0x%02X (DMP only)\r\n", bits);
    }

    return result;
}

inline bool MPU9250Driver::disableInterrupt()
{
    return i2c_->clearBits(
        address_, MPU9250::INT_ENABLE, MPU9250::INT_ENABLE_BITS::RAW_RDY_EN);
}

inline bool MPU9250Driver::configureInterruptPin(
    bool active_low, bool open_drain, bool latch, bool clear_on_read)
{
    uint8_t config = 0;

    if (active_low) {
        config |= MPU9250::INT_PIN_CFG_BITS::ACTL;
    }
    if (open_drain) {
        config |= MPU9250::INT_PIN_CFG_BITS::OPEN;
    }
    if (latch) {
        config |= MPU9250::INT_PIN_CFG_BITS::LATCH_INT_EN;
    }
    if (clear_on_read) {
        // Clear INT on any read (makes it easier to service from main)
        config |= MPU9250::INT_PIN_CFG_BITS::INT_ANYRD_2CLEAR;
    }
    return i2c_->writeRegister(address_, MPU9250::INT_PIN_CFG, config);
}

inline bool MPU9250Driver::initializeMagnetometer()
{
    logsys::printf("[MPU9250] Initializing magnetometer...\r\n");

    if (!i2c_->setBits(address_, MPU9250::INT_PIN_CFG,
            MPU9250::INT_PIN_CFG_BITS::BYPASS_EN)) {
        return false;
    }
    HAL_Delay(10);

    uint8_t mag_id;
    if (!readMagRegister(MPU9250::AK8963::WIA, mag_id)) {
        return false;
    }

    if (mag_id != MPU9250::AK8963_WHO_AM_I_VALUE) {
        logsys::printf(
            "[MPU9250] Magnetometer WHO_AM_I failed: 0x%02X\r\n", mag_id);
        return false;
    }

    writeMagRegister(MPU9250::AK8963::CNTL1, 0x00);
    HAL_Delay(10);

    writeMagRegister(MPU9250::AK8963::CNTL1,
        static_cast<uint8_t>(MPU9250::AK8963_BITS::MODE::FUSE_ROM_ACCESS));
    HAL_Delay(10);

    uint8_t asa[3];
    readMagRegisters(MPU9250::AK8963::ASAX, asa, 3);

    mag_scale_x_ = ((static_cast<float>(asa[0]) - 128.0f) / 256.0f + 1.0f)
        * MPU9250::SENSITIVITY::MAG_16BIT;
    mag_scale_y_ = ((static_cast<float>(asa[1]) - 128.0f) / 256.0f + 1.0f)
        * MPU9250::SENSITIVITY::MAG_16BIT;
    mag_scale_z_ = ((static_cast<float>(asa[2]) - 128.0f) / 256.0f + 1.0f)
        * MPU9250::SENSITIVITY::MAG_16BIT;

    writeMagRegister(MPU9250::AK8963::CNTL1, 0x00);
    HAL_Delay(10);

    uint8_t mag_config
        = static_cast<uint8_t>(MPU9250::AK8963_BITS::MODE::CONT_MEASURE_2)
        | static_cast<uint8_t>(MPU9250::AK8963_BITS::OUTPUT_BITS::BIT_16);
    writeMagRegister(MPU9250::AK8963::CNTL1, mag_config);
    HAL_Delay(10);

    logsys::printf("[MPU9250] Magnetometer initialized\r\n");
    return true;
}

inline bool MPU9250Driver::enableMagnetometer()
{
    mag_enabled_ = initializeMagnetometer();
    return mag_enabled_;
}

inline bool MPU9250Driver::disableMagnetometer()
{
    mag_enabled_ = false;
    return writeMagRegister(MPU9250::AK8963::CNTL1, 0x00);
}

inline bool MPU9250Driver::calibrateGyro(uint16_t samples)
{
    logsys::printf("[MPU9250] Calibrating gyro (%d samples)...\r\n", samples);

    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    for (uint16_t i = 0; i < samples; i++) {
        Vector3 gyro;
        if (!readGyro(gyro)) {
            return false;
        }

        sum_x += static_cast<int32_t>(gyro.x * gyro_scale_);
        sum_y += static_cast<int32_t>(gyro.y * gyro_scale_);
        sum_z += static_cast<int32_t>(gyro.z * gyro_scale_);

        HAL_Delay(2);
    }

    int16_t offset_x = -(sum_x / samples);
    int16_t offset_y = -(sum_y / samples);
    int16_t offset_z = -(sum_z / samples);

    logsys::printf("[MPU9250] Gyro bias: X=%d, Y=%d, Z=%d\r\n", offset_x,
        offset_y, offset_z);

    return setGyroBias(offset_x, offset_y, offset_z);
}

inline bool MPU9250Driver::calibrateAccel(uint16_t samples)
{
    // Placeholder - requires knowing device orientation
    return true;
}

inline bool MPU9250Driver::setGyroBias(
    int16_t offset_x, int16_t offset_y, int16_t offset_z)
{
    uint8_t data[2];

    data[0] = (offset_x >> 8) & 0xFF;
    data[1] = offset_x & 0xFF;
    i2c_->writeRegisters(address_, MPU9250::XG_OFFSET_H, data, 2);

    data[0] = (offset_y >> 8) & 0xFF;
    data[1] = offset_y & 0xFF;
    i2c_->writeRegisters(address_, MPU9250::YG_OFFSET_H, data, 2);

    data[0] = (offset_z >> 8) & 0xFF;
    data[1] = offset_z & 0xFF;
    i2c_->writeRegisters(address_, MPU9250::ZG_OFFSET_H, data, 2);

    return true;
}
