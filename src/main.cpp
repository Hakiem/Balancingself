#include "main.h"
#include "mpu9250.hpp"
#include "MPU9250_regs.hpp"
#include "board.hpp"
#include "command_handler.hpp"
#include "console.hpp"
#include "i2c_bridge.hpp"
#include "logger.hpp"
#include <cmath>

extern "C" {
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;
SPI_HandleTypeDef hspi1;
}

namespace
{
constexpr uint32_t kBlinkIntervalMs = 1000; // LED blink interval
} // namespace

// Add these global variables or in a struct/class as needed
float accel_bias[3] = { 0 }, gyro_bias[3] = { 0 }, mag_bias[3] = { 0 };

// Helper function to compute average bias
void calibrate_bias(MPU9250& imu, int samples = 50)
{
    float accel_sum[3] = { 0 }, gyro_sum[3] = { 0 }, mag_sum[3] = { 0 };
    for (int i = 0; i < samples; ++i) {
        float a[3], g[3], m[3];
        imu.readAccel(a);
        imu.readGyro(g);
        imu.readMag(m);
        for (int j = 0; j < 3; ++j) {
            accel_sum[j] += a[j];
            gyro_sum[j] += g[j];
            mag_sum[j] += m[j];
        }
        HAL_Delay(10);
    }
    for (int j = 0; j < 3; ++j) {
        accel_bias[j] = accel_sum[j] / samples;
        gyro_bias[j] = gyro_sum[j] / samples;
        mag_bias[j] = mag_sum[j] / samples;
    }
}

// Helper function to compute pitch, roll, yaw
void compute_orientation(float* accel, float* gyro, float* mag, float& pitch,
    float& roll, float& yaw)
{
    // Remove bias
    for (int i = 0; i < 3; ++i) {
        accel[i] -= accel_bias[i];
        gyro[i] -= gyro_bias[i];
        mag[i] -= mag_bias[i];
    }
    // Calculate roll and pitch from accelerometer
    roll = atan2(accel[1], accel[2]) * 180.0f / M_PI;
    pitch = atan(-accel[0] / sqrt(accel[1] * accel[1] + accel[2] * accel[2]))
        * 180.0f / M_PI;
    // Yaw from magnetometer (compensate for tilt)
    float mag_x
        = mag[0] * cos(pitch * M_PI / 180) + mag[2] * sin(pitch * M_PI / 180);
    float mag_y = mag[0] * sin(roll * M_PI / 180) * sin(pitch * M_PI / 180)
        + mag[1] * cos(roll * M_PI / 180)
        - mag[2] * sin(roll * M_PI / 180) * cos(pitch * M_PI / 180);
    yaw = atan2(-mag_y, mag_x) * 180.0f / M_PI;
    if (yaw < 0)
        yaw += 360.0f;
}

int main()
{
    // ========================================================================
    // SYSTEM INITIALIZATION
    // ========================================================================
    HAL_Init();
    SystemClock_Config();

    // Initialize peripherals
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    BlinkyLED();

    // Initialize logging system
    logsys::init(&huart2);

    // Initialize console (UART interface)
    console::initialize(&huart2);
    console::print_banner();
    logsys::printf("[BOOT] System Ready.\r\n");
    logsys::printf("[INFO] Preparing for MPU9250, NRF24L01, DRV8256E\r\n");

    // ========================================================================
    // MPU9250 WHO_AM_I register read test
    // ========================================================================
    I2CBridge i2c(&hi2c1);
    uint8_t whoami = 0;
    bool ok = i2c.readRegister(
        MPU9250::I2C_ADDR_AD0_LOW, MPU9250::WHO_AM_I, whoami);
    if (ok) {
        logsys::printf("[MPU9250] WHO_AM_I = 0x%02X\r\n", whoami);
    } else {
        logsys::printf("[MPU9250] WHO_AM_I read failed!\r\n");
    }

    // Initialize MPU9250 object
    MPU9250 imu(&hi2c1);

    // Calibrate bias at startup
    calibrate_bias(imu, 50);

    // ========================================================================
    // MAIN LOOP
    // ========================================================================
    uint32_t last_blink = 0;

    while (true) {
        // Handle incoming UART commands
        console::handle_input();

        // Process commands when ready
        if (console::command_ready()) {
            command_handler::process_command(console::get_command_buffer());
            console::clear_command();
        }

        // Read sensors
        float accel[3], gyro[3], mag[3];
        imu.readAccel(accel);
        imu.readGyro(gyro);
        imu.readMag(mag);

        // Compute orientation
        float pitch, roll, yaw;
        compute_orientation(accel, gyro, mag, pitch, roll, yaw);

        // Now you can use pitch, roll, yaw
        // logsys::printf("P:%.2f R:%.2f Y:%.2f\r\n", pitch, roll, yaw);

        // Blink heartbeat LED
        const uint32_t now = HAL_GetTick();
        if (now - last_blink >= kBlinkIntervalMs) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            last_blink = now;
        }

        // Short delay to prevent busy-waiting
        HAL_Delay(1);
    }
}
