#include "main.h"
#include "MPU9250_regs.hpp"
#include "mpu9250.hpp"
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

volatile bool g_imu_int_flag = false;

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

    MPU9250Driver imu(&i2c);

    bool ready_low = i2c.isDeviceReady(MPU9250::I2C_ADDR_AD0_LOW);
    logsys::printf("[MPU9250] Addr 0x68 ready: %s (HAL=%d)\r\n",
        ready_low ? "YES" : "NO", i2c.lastStatus());
    bool ready_high = i2c.isDeviceReady(MPU9250::I2C_ADDR_AD0_HIGH);
    logsys::printf("[MPU9250] Addr 0x69 ready: %s (HAL=%d)\r\n",
        ready_high ? "YES" : "NO", i2c.lastStatus());

    uint8_t whoami = 0;
    bool ok = i2c.readRegister(
        MPU9250::I2C_ADDR_AD0_LOW, MPU9250::WHO_AM_I, whoami);
    logsys::printf("[MPU9250] WHO_AM_I @0x68 %s (0x%02X, HAL=%d)\r\n",
        ok ? "OK" : "FAIL", whoami, i2c.lastStatus());

    imu.initialize();
    imu.initializeMagnetometer();
    logsys::printf("[MPU9250] Calibrating gyro (keep still)...\r\n");
    imu.calibrateGyro(500);
    logsys::printf("[MPU9250] Gyro calibration done\r\n");
    logsys::printf("[MPU9250] Calibrating magnetometer (move in figure 8)...\r\n");
    imu.calibrateMag(300, 20);
    logsys::printf("[MPU9250] Magnetometer calibration done\r\n");

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

        if (g_imu_int_flag) {
            g_imu_int_flag = false;
            if (imu.dataReady() && imu.update()) {
                const auto& e = imu.getEulerAngles();
                logsys::printf("[IMU] roll=%.2f pitch=%.2f yaw=%.2f\r\n",
                            e.roll, e.pitch, e.yaw);
            }
        }

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
