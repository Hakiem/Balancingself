/**
 * @file mpu9250.hpp
 * @brief MPU9250 9-axis IMU driver (header-only)
 *
 * High-level driver for MPU9250 that reads raw gyro/accel/mag data and
 * computes basic roll, pitch, and yaw angles in firmware (no on-chip fusion).
 *
 * Key features:
 * - Raw sensor access (gyro, accel, mag, temperature)
 * - Simple roll/pitch/yaw calculation (tilt + mag heading)
 * - Configurable sample rates and ranges
 * - FIFO and interrupt support
 */

#pragma once

#include "MPU9250_regs.hpp"
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
 * @brief Complete MPU9250 driver
 *
 * Usage example:
 *   MPU9250 imu(&i2c_bridge, MPU9250::I2C_ADDR_AD0_LOW);
 *   imu.initialize();
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
        bool int_active_low;
        bool int_open_drain;
        bool int_latch;
        bool int_clear_on_read;

        Config()
            : gyro_fsr(MPU9250::GYRO_CONFIG_BITS::FS_SEL::DPS_2000)
            , accel_fsr(MPU9250::ACCEL_CONFIG_BITS::AFS_SEL::G_2)
            , dlpf(MPU9250::CONFIG_BITS::DLPF::BW_20HZ)
            , sample_rate_hz(200) // default 200 Hz
            , enable_magnetometer(false)
            , enable_interrupts(true)
            , int_active_low(true)
            , int_open_drain(false)
            , int_latch(true)
            , int_clear_on_read(true)
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
        , mag_enabled_(false)
        , gyro_scale_(MPU9250::SENSITIVITY::GYRO_2000DPS)
        , accel_scale_(MPU9250::SENSITIVITY::ACCEL_2G)
        , last_update_ms_(0)
    {
        mag_user_offset_ = Vector3(0.0f, 0.0f, 0.0f);
        mag_user_scale_ = Vector3(1.0f, 1.0f, 1.0f);
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
    // Data Acquisition
    // ========================================================================

    /**
     * @brief Check if new data is available
     * @return true if data ready
     */
    bool dataReady();

    /**
     * @brief Update sensor data from gyro/accel/mag
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
     * @brief Calibrate magnetometer (hard/soft iron)
     * Collects samples while you move the board in figure-8s.
     * @param samples Number of samples to collect
     * @param delay_ms Delay between samples
     * @return true if successful
     */
    bool calibrateMag(uint16_t samples = 300, uint16_t delay_ms = 20);

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

    // State
    bool mag_enabled_;

    // Sensor data
    Quaternion quat_;
    EulerAngles euler_;
    Vector3 gyro_;
    Vector3 accel_;
    Vector3 mag_;
    float temperature_;
    Vector3 mag_user_offset_;
    Vector3 mag_user_scale_;

    // Scale factors
    float gyro_scale_;
    float accel_scale_;
    float mag_scale_x_;
    float mag_scale_y_;
    float mag_scale_z_;

    // Error tracking
    const char* last_error_;

    uint32_t last_update_ms_;

    void updateOrientation(
        const Vector3& gyro, const Vector3& accel, const Vector3* mag,
        float dt_ms);
    void eulerToQuaternion();

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
        if (!configureInterruptPin(config.int_active_low,
                config.int_open_drain, config.int_latch,
                config.int_clear_on_read)) {
            last_error_ = "Interrupt config failed";
            return false;
        }

        uint8_t int_status = 0;
        i2c_->readRegister(address_, MPU9250::INT_STATUS, int_status);

        if (!enableInterrupt()) {
            last_error_ = "Interrupt enable failed";
            return false;
        }

        uint8_t int_cfg = 0;
        uint8_t int_en = 0;
        i2c_->readRegister(address_, MPU9250::INT_PIN_CFG, int_cfg);
        i2c_->readRegister(address_, MPU9250::INT_ENABLE, int_en);
        logsys::printf(
            "[MPU9250] INT cfg=0x%02X en=0x%02X status=0x%02X\r\n", int_cfg,
            int_en, int_status);
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
inline bool MPU9250Driver::dataReady()
{
    uint8_t status;
    if (!i2c_->readRegister(address_, MPU9250::INT_STATUS, status)) {
        return false;
    }

    return (status & MPU9250::INT_STATUS_BITS::RAW_DATA_RDY_INT) != 0;
}

inline bool MPU9250Driver::update()
{
    Vector3 gyro;
    Vector3 accel;

    if (!readGyro(gyro) || !readAccel(accel)) {
        return false;
    }

    Vector3 mag;
    const Vector3* mag_ptr = nullptr;
    if (mag_enabled_ && readMag(mag)) {
        mag_ptr = &mag;
    }

    const uint32_t now_ms = HAL_GetTick();
    float dt_ms = 0.0f;
    if (last_update_ms_ != 0) {
        dt_ms = static_cast<float>(now_ms - last_update_ms_);
    }
    last_update_ms_ = now_ms;

    updateOrientation(gyro, accel, mag_ptr, dt_ms);
    return true;
}

inline void MPU9250Driver::updateOrientation(
    const Vector3& gyro, const Vector3& accel, const Vector3* mag,
    float dt_ms)
{
    // Use a copy so we can flip sign when Z is inverted (sensor upside-down)
    float ax = accel.x;
    float ay = accel.y;
    float az = accel.z;

    if (az < 0.0f) {
        ax = -ax;
        ay = -ay;
        az = -az;
    }

    const float roll_rad = atan2f(ay, az);
    const float pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));

    float yaw_deg = euler_.yaw;
    if (mag) {
        const float cr = cosf(roll_rad);
        const float sr = sinf(roll_rad);
        const float cp = cosf(pitch_rad);
        const float sp = sinf(pitch_rad);

        const float mx = mag->x;
        const float my = mag->y;
        const float mz = mag->z;

        const float Xh = mx * cp + mz * sp;
        const float Yh = mx * sr * sp + my * cr - mz * sr * cp;

        yaw_deg = atan2f(-Yh, Xh) * 180.0f / M_PI;
    } else if (dt_ms > 0.0f) {
        // No mag update available; integrate gyro Z to maintain heading
        yaw_deg += gyro.z * (dt_ms / 1000.0f);
    }

    while (yaw_deg < 0.0f) {
        yaw_deg += 360.0f;
    }
    while (yaw_deg >= 360.0f) {
        yaw_deg -= 360.0f;
    }

    euler_.roll = roll_rad * 180.0f / M_PI;
    euler_.pitch = pitch_rad * 180.0f / M_PI;
    euler_.yaw = yaw_deg;

    eulerToQuaternion();
}

inline void MPU9250Driver::eulerToQuaternion()
{
    const float half_roll = euler_.roll * (M_PI / 180.0f) * 0.5f;
    const float half_pitch = euler_.pitch * (M_PI / 180.0f) * 0.5f;
    const float half_yaw = euler_.yaw * (M_PI / 180.0f) * 0.5f;

    const float cr = cosf(half_roll);
    const float sr = sinf(half_roll);
    const float cp = cosf(half_pitch);
    const float sp = sinf(half_pitch);
    const float cy = cosf(half_yaw);
    const float sy = sinf(half_yaw);

    quat_.w = cr * cp * cy + sr * sp * sy;
    quat_.x = sr * cp * cy - cr * sp * sy;
    quat_.y = cr * sp * cy + sr * cp * sy;
    quat_.z = cr * cp * sy - sr * sp * cy;
    quat_.normalize();
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

    // Apply factory ASA scaling
    mag.x = static_cast<float>(raw_x) * mag_scale_x_;
    mag.y = static_cast<float>(raw_y) * mag_scale_y_;
    mag.z = static_cast<float>(raw_z) * mag_scale_z_;

    // Apply user hard-iron offset and soft-iron scale if present
    mag.x = (mag.x - mag_user_offset_.x) * mag_user_scale_.x;
    mag.y = (mag.y - mag_user_offset_.y) * mag_user_scale_.y;
    mag.z = (mag.z - mag_user_offset_.z) * mag_user_scale_.z;

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
    const uint8_t bits = MPU9250::INT_ENABLE_BITS::RAW_RDY_EN;
    const bool result
        = i2c_->writeRegister(address_, MPU9250::INT_ENABLE, bits);

    if (result) {
        logsys::printf("[MPU9250] INT_ENABLE = 0x%02X\r\n", bits);
    }

    return result;
}

inline bool MPU9250Driver::disableInterrupt()
{
    return i2c_->writeRegister(address_, MPU9250::INT_ENABLE, 0x00);
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
    mag_enabled_ = true;
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

inline bool MPU9250Driver::calibrateMag(uint16_t samples, uint16_t delay_ms)
{
    if (!mag_enabled_) {
        return false;
    }

    Vector3 mag_min(1e6f, 1e6f, 1e6f);
    Vector3 mag_max(-1e6f, -1e6f, -1e6f);

    for (uint16_t i = 0; i < samples; ++i) {
        Vector3 m;
        if (readMag(m)) {
            mag_min.x = fminf(mag_min.x, m.x);
            mag_min.y = fminf(mag_min.y, m.y);
            mag_min.z = fminf(mag_min.z, m.z);

            mag_max.x = fmaxf(mag_max.x, m.x);
            mag_max.y = fmaxf(mag_max.y, m.y);
            mag_max.z = fmaxf(mag_max.z, m.z);
        }
        HAL_Delay(delay_ms);
    }

    Vector3 offset((mag_min.x + mag_max.x) * 0.5f,
        (mag_min.y + mag_max.y) * 0.5f, (mag_min.z + mag_max.z) * 0.5f);

    Vector3 scale_delta((mag_max.x - mag_min.x) * 0.5f,
        (mag_max.y - mag_min.y) * 0.5f, (mag_max.z - mag_min.z) * 0.5f);

    const float avg_delta
        = (scale_delta.x + scale_delta.y + scale_delta.z) / 3.0f;

    Vector3 scale(1.0f, 1.0f, 1.0f);
    if (scale_delta.x > 0.0f && scale_delta.y > 0.0f && scale_delta.z > 0.0f) {
        scale.x = avg_delta / scale_delta.x;
        scale.y = avg_delta / scale_delta.y;
        scale.z = avg_delta / scale_delta.z;
    }

    mag_user_offset_ = offset;
    mag_user_scale_ = scale;

    logsys::printf("[MPU9250] Mag calib offset: %.2f %.2f %.2f\r\n",
        offset.x, offset.y, offset.z);
    logsys::printf("[MPU9250] Mag calib scale: %.3f %.3f %.3f\r\n", scale.x,
        scale.y, scale.z);

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
