#include "main.h"
#include "board.hpp"
#include "command_handler.hpp"
#include "console.hpp"
#include "logger.hpp"

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
    // TODO: Hardware initialization will go here:
    // - MPU9250 IMU (I2C or SPI)
    // - NRF24L01 wireless (SPI)
    // - DRV8256E motor drivers (PWM + GPIO)
    // - Encoder inputs (Timer in encoder mode)
    // ========================================================================

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

        // TODO: Main control loop tasks
        // - Read IMU data (MPU9250)
        // - Read encoder positions
        // - Run balancing PID controller
        // - Update motor PWM outputs
        // - Handle wireless communication (NRF24L01)

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
