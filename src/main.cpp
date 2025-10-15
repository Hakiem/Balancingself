#include "main.h"
#include "board.h"
#include "logger.hpp"
#include "motion_profile.hpp"
#include "uart_bridge.hpp"
#include <cstdlib>
#include <cstring>

extern "C" {
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1; // For TMC2208 Half-Duplex
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;
}

namespace
{
constexpr uint32_t kBlinkIntervalMs = 1000U;
constexpr char kHelloMsg[] = "\nHello World!\r\n\n";

void send_uart2(const char* msg, size_t len)
{
    HAL_UART_Transmit(&huart2,
        reinterpret_cast<uint8_t*>(const_cast<char*>(msg)),
        static_cast<uint16_t>(len), HAL_MAX_DELAY);
}

// Simple command buffer and parser
char cmd_buffer[128];
uint8_t cmd_index = 0;
bool cmd_ready = false;

// Non-blocking command input handler
void handle_uart_input()
{
    uint8_t byte;
    if (HAL_UART_Receive(&huart2, &byte, 1, 0) == HAL_OK) {
        if (byte == '\r' || byte == '\n') {
            if (cmd_index > 0) {
                cmd_buffer[cmd_index] = '\0';
                cmd_ready = true;
            }
        } else if (byte >= 32 && byte <= 126
            && cmd_index < sizeof(cmd_buffer) - 1) {
            cmd_buffer[cmd_index++] = byte;
            // Echo character back
            HAL_UART_Transmit(&huart2, &byte, 1, HAL_MAX_DELAY);
        }
    }
}

// Command parser and executor
void process_command(motion::MotionProfile& mp)
{
    if (!cmd_ready)
        return;

    cmd_ready = false;
    logsys::printf("\r\n[CMD] %s\r\n", cmd_buffer);

    // Parse command
    char* token = strtok(cmd_buffer, " ");
    if (!token) {
        cmd_index = 0;
        return;
    }

    if (strcmp(token, "scurve") == 0) {
        // Format: scurve <duration> <peak_speed> [forward/reverse]
        float duration = 5.0f;
        float peak_speed = 10000.0f;
        motion::Direction dir = motion::Direction::Forward;

        token = strtok(nullptr, " ");
        if (token)
            duration = atof(token);

        token = strtok(nullptr, " ");
        if (token)
            peak_speed = atof(token);

        token = strtok(nullptr, " ");
        if (token && strcmp(token, "reverse") == 0) {
            dir = motion::Direction::Reverse;
        }

        logsys::printf("[RUN] S-curve: %.1fs, %.0f usteps/s, %s\r\n", duration,
            peak_speed,
            (dir == motion::Direction::Forward) ? "forward" : "reverse");

        mp.runSCurve(duration, peak_speed, dir, 10);
        mp.stop();
        logsys::printf("[RUN] S-curve done.\r\n");

    } else if (strcmp(token, "triangle") == 0) {
        // Format: triangle <duration> <peak_speed> [forward/reverse]
        float duration = 5.0f;
        float peak_speed = 10000.0f;
        motion::Direction dir = motion::Direction::Forward;

        token = strtok(nullptr, " ");
        if (token)
            duration = atof(token);

        token = strtok(nullptr, " ");
        if (token)
            peak_speed = atof(token);

        token = strtok(nullptr, " ");
        if (token && strcmp(token, "reverse") == 0) {
            dir = motion::Direction::Reverse;
        }

        logsys::printf("[RUN] Triangle: %.1fs, %.0f usteps/s, %s\r\n", duration,
            peak_speed,
            (dir == motion::Direction::Forward) ? "forward" : "reverse");

        mp.runTriangular(duration, peak_speed, dir, 10);
        mp.stop();
        logsys::printf("[RUN] Triangle done.\r\n");

    } else if (strcmp(token, "stop") == 0) {
        mp.stop();
        logsys::printf("[RUN] Motor stopped.\r\n");

    } else if (strcmp(token, "help") == 0) {
        logsys::printf("Commands:\r\n");
        logsys::printf(
            "  scurve <duration> <peak_speed> [forward/reverse]\r\n");
        logsys::printf(
            "  triangle <duration> <peak_speed> [forward/reverse]\r\n");
        logsys::printf("  stop\r\n");
        logsys::printf("  help\r\n");
        logsys::printf("Example: scurve 3.0 15000 forward\r\n");

    } else {
        logsys::printf("[ERR] Unknown command. Type 'help' for usage.\r\n");
    }

    cmd_index = 0;
    logsys::printf("\r\n> ");
}
} // namespace

// Optional: a quick banner on UART2
static void print_banner()
{
    static const char msg[] = "\r\n=== TMC2208 UART-only demo ===\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
}

int main(void)
{
    HAL_Init();

    SystemClock_Config();
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_GPIO_Init();
    BlinkyLED();

    send_uart2(kHelloMsg, sizeof(kHelloMsg) - 1);

    // -------- Logging on UART2 --------
    logsys::init(&huart2);
    print_banner();
    logsys::printf("[BOOT] System up.\r\n");

    // -------- Bridge → Transport → Device --------
    UART_Bridge bridge(&huart1); // wraps half-duplex TX/RX + hex dump
    TMC2208_UART_Transport::Config tcfg {};
    tcfg.retries = 1; // retry a couple times on CRC/short RX
    tcfg.read_timeout_us = 4000;
    tcfg.verify_write = false; // set true if you want RW readback verify
    TMC2208_UART_Transport bus(&bridge, /*slave*/ 0x00, tcfg);
    TMC2208::Device drv(bus);

    // -------- Minimal driver configuration --------
    // For responsive torque (recommended for balance/quick moves), use
    // SpreadCycle:
    drv.gconf_or(TMC2208::gconf::EN_SPREADCYCLE);
    drv.chop_set_mres(TMC2208::chopconf::_16);
    drv.chop_set_toff(3);
    drv.chop_set_hstrt(5);
    drv.chop_set_hend(7);
    drv.chop_set_vsense(false); // VSENSE=0 → higher full-scale current

    // If you want ultra-quiet bench tests instead, comment the block above and
    // enable StealthChop: drv.gconf_andc(TMC2208::gconf::EN_SPREADCYCLE);
    // drv.chop_set_mres(TMC2208::chopconf::_16);
    // drv.chop_set(TMC2208::chopconf::INTPOL);
    // drv.pwm_set_autoscale(true);
    // drv.pwm_set_autograd(true);
    // drv.pwm_set_freq(TMC2208::pwmconf::FREQ_12);
    // drv.pwm_set_ofs(0x10);
    // drv.pwm_set_grad(0x40);
    // drv.tpwmthrs(1000);

    // Current limits (tune to your motor + Rsense)
    drv.ihold_irun(/*IHOLD=*/8, /*IRUN=*/24, /*IHOLDDELAY=*/8);
    drv.tpowerdown(20); // drop to IHOLD after 20*~12ms idle

    logsys::printf("[INFO] TMC2208 configured.\r\n");

    // -------- Motion: S-curve using VACTUAL (UART-only; no STEP/DIR) --------
    motion::MotionProfile mp(drv);
    // IMPORTANT: this clock is the TMC2208's clock, not MCU 64 MHz.
    // Keep 12 MHz unless you physically feed an external CLK into the driver.
    mp.setClockHz(12'000'000.0f);

    logsys::printf("[READY] Type 'help' for commands.\r\n");
    logsys::printf("> ");

    while (1) {
        // Handle incoming commands
        handle_uart_input();
        process_command(mp);

        // Blink LED to show system is alive
        static uint32_t last_blink = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_blink >= kBlinkIntervalMs) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
            last_blink = now;
        }

        HAL_Delay(1); // Small delay to prevent overwhelming the system
    }

    return 0;
}