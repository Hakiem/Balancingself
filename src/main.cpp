#include "main.h"
#include "board.hpp"
#include "command_handler.hpp"
#include "console.hpp"
#include "logger.hpp"
#include "motor_manager.hpp"

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
    logsys::printf("[BOOT] Ready.\r\n");

    // ========================================================================
    // TMC5160 DRIVER INITIALIZATION
    // ========================================================================

    // Enable driver power and wait for power-up
    TMC_DriversEnable(true);
    HAL_Delay(500); // 500ms power-up delay

    // Configure SPI at lower speed for initialization (safer)
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }

    // Initialize all motor contexts
    motor_manager::initialize();

    // After initialization, increase SPI speed for normal operation
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }

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

        // Service timed motor moves
        motor_manager::service_timed_moves();

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
