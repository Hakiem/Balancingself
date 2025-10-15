#include "main.h"
#include "board.h"
#include "logger.hpp"
#include "motion_profile.hpp"
#include "uart_bridge.hpp"

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

    const float duration_s = 20.0f; // total time
    const float peak_usteps_s = 12'000.0f; // peak microsteps/s (tune this)
    logsys::printf("[RUN] S-curve: %.1fs, peak %.0f usteps/s\r\n", duration_s,
        peak_usteps_s);

    (void)mp.runSCurve(
        duration_s, peak_usteps_s, motion::Direction::Forward, /*update*/ 10);
    mp.stop();
    logsys::printf("[RUN] Done.\r\n");

    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        HAL_Delay(kBlinkIntervalMs);
    }

    return 0;
}