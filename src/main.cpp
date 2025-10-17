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
UART_HandleTypeDef huart1; // For TMC2209 Half-Duplex
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

// Continuous spinning state
bool continuous_spinning = false;
motion::Direction spin_direction = motion::Direction::Forward;
float current_spin_speed
    = 0.0f; // Track current spinning speed for smooth deceleration

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
void process_command(motion::MotionProfile& mp, TMC2208::Device& drv,
    TMC22xx_UART_Transport& transport)
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

    } else if (strcmp(token, "test") == 0) {
        // Simple test: short forward movement
        logsys::printf("[TEST] Running short test movement...\r\n");
        mp.runSCurve(1.0f, 5000.0f, motion::Direction::Forward, 10);
        mp.stop();
        logsys::printf(
            "[TEST] Test done. Motor should have moved slightly.\r\n");

    } else if (strcmp(token, "current") == 0) {
        // Show current settings (configured values, not read from chip)
        logsys::printf("[INFO] Configured: IHOLD=8, IRUN=24, IHOLDDELAY=8\r\n");
        logsys::printf("[INFO] Note: IHOLD_IRUN is write-only register\r\n");

    } else if (strcmp(token, "disable") == 0) {
        // Disable motor to reduce power consumption
        drv.ihold_irun(0, 0, 0);
        continuous_spinning = false;
        current_spin_speed = 0.0f;
        logsys::printf(
            "[INFO] Motor disabled - current should drop to ~0mA\r\n");

    } else if (strcmp(token, "enable") == 0) {
        // Re-enable motor with original settings
        drv.ihold_irun(8, 24, 8);
        logsys::printf("[INFO] Motor re-enabled with IHOLD=8, IRUN=24\r\n");

    } else if (strcmp(token, "turbo") == 0) {
        // Set maximum current for highest speed capability
        drv.ihold_irun(8, 31, 8);
        logsys::printf("[INFO] TURBO MODE: IHOLD=8, IRUN=31 (MAX CURRENT)\r\n");
        logsys::printf(
            "[WARN] Motor will run hotter - monitor temperature!\r\n");

    } else if (strcmp(token, "spin") == 0) {
        // Format: spin [forward/reverse] - Continuous spinning at max speed
        motion::Direction dir = motion::Direction::Forward;

        token = strtok(nullptr, " ");
        if (token && strcmp(token, "reverse") == 0) {
            dir = motion::Direction::Reverse;
        }

        continuous_spinning = true;
        spin_direction = dir;

        // Use a much lower speed to avoid motor stalling/noise
        const float spin_speed
            = 8000.0f; // microsteps/s - reduced for reliability
        current_spin_speed = spin_speed; // Store for deceleration

        logsys::printf(
            "[SPIN] Starting %s spin with gradual acceleration...\r\n",
            (dir == motion::Direction::Forward) ? "forward" : "reverse");

        // Gradually ramp up to target speed to avoid stalling
        const int steps = 20; // Number of acceleration steps
        const uint32_t step_delay_ms
            = 50; // 50ms between steps = 1 second total ramp

        for (int i = 1; i <= steps; i++) {
            float current_speed = (spin_speed * i) / steps;
            const int32_t vactual = mp.vactualFromUSteps(
                (dir == motion::Direction::Forward) ? current_speed
                                                    : -current_speed);

            if (!drv.vactual(vactual)) {
                logsys::printf("[ERR] Failed to set speed step %d\r\n", i);
                continuous_spinning = false;
                return;
            }
            HAL_Delay(step_delay_ms);
        }

        logsys::printf("[SPIN] Continuous %s spinning at %.0f usteps/s\r\n",
            (dir == motion::Direction::Forward) ? "forward" : "reverse",
            spin_speed);
        logsys::printf(
            "[SPIN] Send any command to stop (e.g., 'stop', 'x')\r\n");

    } else if (strcmp(token, "turbo-spin") == 0) {
        // Format: turbo-spin [forward/reverse] - Maximum speed spinning
        motion::Direction dir = motion::Direction::Forward;

        token = strtok(nullptr, " ");
        if (token && strcmp(token, "reverse") == 0) {
            dir = motion::Direction::Reverse;
        }

        // First set maximum current for highest torque capability
        drv.ihold_irun(8, 31, 8);
        logsys::printf("[TURBO] Setting maximum current (IRUN=31)...\r\n");
        HAL_Delay(100); // Let current setting stabilize

        continuous_spinning = true;
        spin_direction = dir;

        // Use maximum safe speed with high current
        const float turbo_speed
            = 40000.0f; // microsteps/s - pushing towards motor limits!
        current_spin_speed = turbo_speed; // Store for deceleration

        logsys::printf(
            "[TURBO] Starting %s TURBO spin with gradual acceleration...\r\n",
            (dir == motion::Direction::Forward) ? "forward" : "reverse");
        logsys::printf(
            "[WARN] Motor will run hotter - monitor temperature!\r\n");

        // Gradually ramp up to turbo speed
        const int steps = 25; // More steps for higher speed
        const uint32_t step_delay_ms
            = 60; // 60ms between steps = 1.5 second total ramp

        for (int i = 1; i <= steps; i++) {
            float current_speed = (turbo_speed * i) / steps;
            const int32_t vactual = mp.vactualFromUSteps(
                (dir == motion::Direction::Forward) ? current_speed
                                                    : -current_speed);

            if (!drv.vactual(vactual)) {
                logsys::printf(
                    "[ERR] Failed to set turbo speed step %d\r\n", i);
                continuous_spinning = false;
                return;
            }
            HAL_Delay(step_delay_ms);
        }

        logsys::printf(
            "[TURBO] Continuous %s TURBO spinning at %.0f usteps/s\r\n",
            (dir == motion::Direction::Forward) ? "forward" : "reverse",
            turbo_speed);
        logsys::printf(
            "[TURBO] Send any command to stop (e.g., 'stop', 'x')\r\n");

    } else if (strcmp(token, "x") == 0 || strcmp(token, "stop") == 0) {
        if (continuous_spinning) {
            continuous_spinning = false;

            // Get current speed for gradual deceleration
            const float decel_from_speed = current_spin_speed;

            logsys::printf("[SPIN] Gradually decelerating from %.0f usteps/s "
                           "to stop...\r\n",
                decel_from_speed);

            // Gradually ramp down from current speed to zero
            const int steps
                = 15; // Fewer steps for quicker stop, but still smooth
            const uint32_t step_delay_ms
                = 40; // 40ms between steps = 0.6 second total decel

            for (int i = steps - 1; i >= 0; i--) {
                float decel_speed = (decel_from_speed * i) / steps;
                const int32_t vactual = mp.vactualFromUSteps(
                    (spin_direction == motion::Direction::Forward)
                        ? decel_speed
                        : -decel_speed);

                if (!drv.vactual(vactual)) {
                    logsys::printf("[ERR] Failed to set decel step %d\r\n", i);
                    break;
                }
                HAL_Delay(step_delay_ms);
            }

            // Final stop
            drv.vactual(0);
            logsys::printf("[SPIN] Continuous spinning stopped smoothly.\r\n");
        } else {
            mp.stop();
            logsys::printf("[RUN] Motor stopped.\r\n");
        }

    } else if (strcmp(token, "scan") == 0) {
        // Scan for TMC2209 drivers at addresses 0x00-0x03
        logsys::printf("[SCAN] Scanning for TMC2209 drivers...\r\n");
        logsys::printf("[SCAN] Address | Status | GCONF Register\r\n");
        logsys::printf("[SCAN] --------|--------|---------------\r\n");

        uint8_t found_count = 0;
        uint8_t original_addr = transport.getSlave();

        for (uint8_t addr = 0x00; addr <= 0x03; addr++) {
            transport.setSlave(addr);
            HAL_Delay(10); // Give time for address change

            // Try to read GCONF register (address 0x00) - always readable
            uint32_t gconf_value = 0;
            bool success = transport.regRead(0x00, gconf_value, 1000);

            if (success) {
                logsys::printf("[SCAN]   0x%02X   |   OK   | 0x%08X\r\n", addr,
                    static_cast<unsigned int>(gconf_value));
                found_count++;

                // Check if this looks like a valid TMC2209 GCONF
                if ((gconf_value & 0xFF) == 0x00
                    || (gconf_value & 0xFF) == 0x01) {
                    logsys::printf(
                        "[SCAN]          | Valid TMC2209 detected!\r\n");
                }
            } else {
                logsys::printf(
                    "[SCAN]   0x%02X   | NO RSP | -----------\r\n", addr);
            }
        }

        // Restore original address
        transport.setSlave(original_addr);

        logsys::printf(
            "[SCAN] Scan complete. Found %u responding driver(s).\r\n",
            found_count);
        if (found_count == 0) {
            logsys::printf(
                "[SCAN] Check wiring: UART_TX, UART_RX, MS1/MS2 pins\r\n");
        } else {
            logsys::printf(
                "[SCAN] Use 'addr <0x00-0x03>' to switch active driver\r\n");
        }

    } else if (strcmp(token, "addr") == 0) {
        // Switch active driver address
        token = strtok(nullptr, " ");
        if (token) {
            uint8_t new_addr = static_cast<uint8_t>(strtol(token, nullptr, 16));
            if (new_addr <= 0x03) {
                uint8_t old_addr = transport.getSlave();
                transport.setSlave(new_addr);
                logsys::printf("[ADDR] Switched from 0x%02X to 0x%02X\r\n",
                    old_addr, new_addr);

                // Quick test to see if driver responds
                uint32_t gconf = 0;
                if (transport.regRead(0x00, gconf, 500)) {
                    logsys::printf(
                        "[ADDR] Driver at 0x%02X responds (GCONF=0x%08X)\r\n",
                        new_addr, static_cast<unsigned int>(gconf));
                } else {
                    logsys::printf(
                        "[ADDR] WARNING: No response from driver at 0x%02X\r\n",
                        new_addr);
                }
            } else {
                logsys::printf("[ERR] Address must be 0x00-0x03\r\n");
            }
        } else {
            logsys::printf(
                "[INFO] Current address: 0x%02X\r\n", transport.getSlave());
            logsys::printf("[INFO] Usage: addr <0x00-0x03>\r\n");
        }

    } else if (strcmp(token, "help") == 0) {
        logsys::printf("Commands:\r\n");
        logsys::printf(
            "  scan - Scan for TMC2209 drivers (addresses 0x00-0x03)\r\n");
        logsys::printf("  addr <0x00-0x03> - Switch active driver address\r\n");
        logsys::printf(
            "  scurve <duration> <peak_speed> [forward/reverse]\r\n");
        logsys::printf(
            "  triangle <duration> <peak_speed> [forward/reverse]\r\n");
        logsys::printf("  spin [forward/reverse] - Continuous smooth spinning "
                       "(8k usteps/s)\r\n");
        logsys::printf("  turbo-spin [forward/reverse] - MAXIMUM speed "
                       "spinning (40k usteps/s)\r\n");
        logsys::printf("  test - Quick movement test\r\n");
        logsys::printf(
            "  turbo - Set maximum current (IRUN=31) for higher speeds\r\n");
        logsys::printf("  stop/x - Stop motor smoothly\r\n");
        logsys::printf("  current - Show current settings\r\n");
        logsys::printf("  disable - Disable motor (0mA consumption)\r\n");
        logsys::printf("  enable - Re-enable motor\r\n");
        logsys::printf("  help - Show this help\r\n");
        logsys::printf("Examples:\r\n");
        logsys::printf("  scurve 3.0 15000 forward\r\n");
        logsys::printf("  spin reverse\r\n");
        logsys::printf("  turbo-spin forward  # Maximum speed!\r\n");

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
    static const char msg[] = "\r\n=== TMC2209 UART demo ===\r\n";
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

    // -------- GPIO Pin Diagnostics --------
    logsys::printf("[TEST] Checking Half-Duplex GPIO configuration...\r\n");
    logsys::printf("[TEST] PA9 (UART1_TX Half-Duplex) mode: %s\r\n",
        (GPIOA->MODER & (3 << (9 * 2))) == (2 << (9 * 2))
            ? "Alternate Function ✓"
            : "WRONG MODE!");
    logsys::printf("[TEST] PA9 Output Type: %s\r\n",
        (GPIOA->OTYPER & (1 << 9)) ? "Open Drain ✓" : "Push-Pull (WRONG!)");
    logsys::printf(
        "[TEST] PA10 should NOT be configured for UART in half-duplex\r\n");

    // Check alternate function registers
    logsys::printf("[TEST] PA9 AF: %u (should be 7 for UART1)\r\n",
        (GPIOA->AFR[1] >> ((9 - 8) * 4)) & 0xF);

    // -------- Test UART1 (PA9/PA10) immediately --------
    logsys::printf("[TEST] Testing UART1 Half-Duplex (PA9 only)...\r\n");

    // Test 1: Direct HAL transmission
    const char uart1_test[]
        = "\x05\x00\x00\xFF\xFF\xFF\xFF\xC0"; // Dummy TMC message
    HAL_StatusTypeDef uart1_status = HAL_UART_Transmit(
        &huart1, (uint8_t*)uart1_test, sizeof(uart1_test) - 1, 1000);

    if (uart1_status == HAL_OK) {
        logsys::printf(
            "[TEST] UART1 HAL TX successful - check PA9 for activity\r\n");
    } else {
        logsys::printf(
            "[ERROR] UART1 HAL TX failed! Status: %d\r\n", uart1_status);
    }

    // Test 2: Multiple bytes to make it visible on logic analyzer
    logsys::printf("[TEST] Sending multiple test bytes on PA9...\r\n");
    for (int i = 0; i < 5; i++) {
        uint8_t test_byte = 0x55; // Alternating pattern 01010101
        HAL_UART_Transmit(&huart1, &test_byte, 1, 100);
        HAL_Delay(10);
    }
    logsys::printf("[TEST] Test pattern sent - should see 5 bytes on PA9\r\n");

    // Give time to observe on logic analyzer
    HAL_Delay(100);

    // -------- Bridge → Transport → Device --------
    UART_Bridge bridge(&huart1); // wraps half-duplex TX/RX + hex dump
    TMC22xx_UART_Transport::Config tcfg {};
    tcfg.retries = 1; // retry a couple times on CRC/short RX
    tcfg.read_timeout_us = 4000;
    tcfg.verify_write = false; // set true if you want RW readback verify
    TMC22xx_UART_Transport bus(&bridge, /*slave*/ 0x00, tcfg);
    TMC2208::Device drv(bus); // TMC2208 API compatible with TMC2209 hardware

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

    logsys::printf("[INFO] TMC2209 configured.\r\n");

    // NOTE: TMC2209 supports UART addressing (used for multi-driver setups)
    // Current config uses default address 0x00 for single driver operation

    // -------- Test TMC UART Communication --------
    logsys::printf("[TEST] Testing TMC UART communication...\r\n");
    uint32_t gconf_test = 0;
    bool comm_test = bus.regRead(0x00, gconf_test, 2000);
    if (comm_test) {
        logsys::printf("[TEST] TMC communication OK! GCONF=0x%08X\r\n",
            static_cast<unsigned int>(gconf_test));
    } else {
        logsys::printf(
            "[ERROR] TMC communication FAILED! Check PA9/PA10 wiring\r\n");
        logsys::printf("[ERROR] This should generate UART activity on PA9\r\n");
    }

    // -------- Motion: S-curve using VACTUAL (UART-only; no STEP/DIR) --------
    motion::MotionProfile mp(drv);
    // IMPORTANT: this clock is the TMC2209's clock, not MCU 64 MHz.
    // Keep 12 MHz unless you physically feed an external CLK into the driver.
    mp.setClockHz(12'000'000.0f);

    logsys::printf("[READY] Type 'help' for commands.\r\n");
    logsys::printf("> ");

    while (1) {
        // Handle incoming commands
        handle_uart_input();
        process_command(mp, drv, bus);

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