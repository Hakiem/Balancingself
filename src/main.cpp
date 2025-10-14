#include "main.h"
#include "board.h"
#include "tmc2208_hal.hpp"

#include <cstdio>
#include <cstdarg>
#include <cstring>

extern "C" {
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;
}

namespace {
constexpr uint32_t kBlinkIntervalMs = 1000U;
constexpr char kHelloMsg[] = "Hello World!\r\n\n\n";
constexpr uint8_t kTmcSlaveAddress = 0; // Node address 0 per TMC2208 default

void send_uart2(const char* msg, size_t len)
{
    HAL_UART_Transmit(&huart2,
        reinterpret_cast<uint8_t*>(const_cast<char*>(msg)),
        static_cast<uint16_t>(len), HAL_MAX_DELAY);
}

void report_tmc_status(const char* action, tmc2208::Driver::Status status)
{
    char msg[80];
    const int n = std::snprintf(msg, sizeof(msg), "%s: %s\r\n", action,
        tmc2208::Driver::status_cstr(status));
    if (n > 0) {
        send_uart2(msg, static_cast<size_t>(n));
    }
}

void send_uart2_fmt(const char* fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    int n = std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n > 0) {
        if (static_cast<size_t>(n) >= sizeof(buf)) {
            n = sizeof(buf) - 1;
        }
        send_uart2(buf, static_cast<size_t>(n));
    }
}

bool read_and_print_reg(
    tmc2208::Driver& tmc, const char* name, uint8_t reg, uint32_t& out)
{
    out = tmc.read_reg(reg);
    if (tmc.last_status() != tmc2208::Driver::Status::Ok) {
        report_tmc_status(name, tmc.last_status());
        return false;
    }
    send_uart2_fmt("%s=0x%08lX\r\n", name, static_cast<unsigned long>(out));
    return true;
}

void print_gstat_details(uint32_t gstat)
{
    if (gstat == 0) {
        send_uart2("  GSTAT flags: none\r\n", sizeof("  GSTAT flags: none\r\n") - 1);
        return;
    }
    send_uart2("  GSTAT flags:\r\n", sizeof("  GSTAT flags:\r\n") - 1);
    if (gstat & tmc2208::GSTAT_RESET) {
        send_uart2("    RESET\r\n", sizeof("    RESET\r\n") - 1);
    }
    if (gstat & tmc2208::GSTAT_DRV_ERR) {
        send_uart2("    DRV_ERR\r\n", sizeof("    DRV_ERR\r\n") - 1);
    }
    if (gstat & tmc2208::GSTAT_UV_CP) {
        send_uart2("    UV_CP\r\n", sizeof("    UV_CP\r\n") - 1);
    }
}

void print_drv_status_details(uint32_t drv_status)
{
    send_uart2("  DRV_STATUS flags:\r\n", sizeof("  DRV_STATUS flags:\r\n") - 1);
    bool any = false;
    auto emit = [&](const char* label) {
        send_uart2("    ", 4);
        send_uart2(label, std::strlen(label));
        send_uart2("\r\n", 2);
        any = true;
    };
    if (drv_status & tmc2208::DRV_STATUS_STST)
        emit("STST (standstill detected)");
    if (drv_status & tmc2208::DRV_STATUS_OTS)
        emit("OTS (overtemp shutdown)");
    if (drv_status & tmc2208::DRV_STATUS_OTPW)
        emit("OTPW (overtemp prewarning)");
    if (drv_status & tmc2208::DRV_STATUS_S2GA)
        emit("S2GA (short to GND on coil A)");
    if (drv_status & tmc2208::DRV_STATUS_S2GB)
        emit("S2GB (short to GND on coil B)");
    if (drv_status & tmc2208::DRV_STATUS_S2VSA)
        emit("S2VSA (short to VS on coil A)");
    if (drv_status & tmc2208::DRV_STATUS_S2VSB)
        emit("S2VSB (short to VS on coil B)");
    if (drv_status & tmc2208::DRV_STATUS_OLA)
        emit("OLA (open load A)");
    if (drv_status & tmc2208::DRV_STATUS_OLB)
        emit("OLB (open load B)");
    if (drv_status & tmc2208::DRV_STATUS_T120)
        emit("Overtemp >=120C");
    if (drv_status & tmc2208::DRV_STATUS_T143)
        emit("Overtemp >=143C");
    if (drv_status & tmc2208::DRV_STATUS_STA)
        emit("STA (StallGuard result available)");
    if (!any) {
        send_uart2("    none\r\n", sizeof("    none\r\n") - 1);
    }
    uint32_t cs_actual = drv_status & 0x1F;
    send_uart2_fmt("    CS_ACTUAL=%lu\r\n", static_cast<unsigned long>(cs_actual));
}

void dump_tmc_diagnostics(tmc2208::Driver& tmc)
{
    send_uart2("== TMC2208 Diagnostics ==\r\n",
        sizeof("== TMC2208 Diagnostics ==\r\n") - 1);

    uint32_t val = 0;

    if (read_and_print_reg(tmc, "  GCONF", tmc2208::REG_GCONF, val)) {
        (void)val;
    }
    if (read_and_print_reg(tmc, "  GSTAT", tmc2208::REG_GSTAT, val)) {
        print_gstat_details(val);
    }
    if (read_and_print_reg(tmc, "  IFCNT", tmc2208::REG_IFCNT, val)) {
        send_uart2_fmt("    IFCNT=%lu (writes acknowledged)\r\n",
            static_cast<unsigned long>(val & tmc2208::IFCNT_MASK));
    }
    if (read_and_print_reg(tmc, "  CHOPCONF", tmc2208::REG_CHOPCONF, val)) {
        (void)val;
    }
    if (read_and_print_reg(tmc, "  DRV_STATUS", tmc2208::REG_DRV_STATUS, val)) {
        print_drv_status_details(val);
    }
    if (read_and_print_reg(tmc, "  PWMCONF", tmc2208::REG_PWMCONF, val)) {
        (void)val;
    }
    if (read_and_print_reg(tmc, "  IHOLD_IRUN", tmc2208::REG_IHOLD_IRUN, val)) {
        uint32_t ihold = val & 0x1F;
        uint32_t irun = (val >> 8) & 0x1F;
        uint32_t iholddelay = (val >> 16) & 0x0F;
        send_uart2_fmt("    IHOLD=%lu IRUN=%lu IHOLDDELAY=%lu\r\n",
            static_cast<unsigned long>(ihold),
            static_cast<unsigned long>(irun),
            static_cast<unsigned long>(iholddelay));
    }
    if (read_and_print_reg(tmc, "  TPOWERDOWN", tmc2208::REG_TPOWERDOWN, val)) {
        send_uart2_fmt("    TPOWERDOWN=%lu (2^18 tCLK units)\r\n",
            static_cast<unsigned long>(val & 0xFFu));
    }
    if (read_and_print_reg(tmc, "  PWM_SCALE", tmc2208::REG_PWM_SCALE, val)) {
        (void)val;
    }
    if (read_and_print_reg(tmc, "  PWM_AUTO", tmc2208::REG_PWM_AUTO, val)) {
        (void)val;
    }
    if (read_and_print_reg(tmc, "  MSCNT", tmc2208::REG_MSCNT, val)) {
        send_uart2_fmt("    microstep count=%lu\r\n",
            static_cast<unsigned long>(val & tmc2208::MSCNT_MASK));
    }
    if (read_and_print_reg(tmc, "  MSCURACT", tmc2208::REG_MSCURACT, val)) {
        int16_t curA = static_cast<int16_t>((val >> 16) & 0x1FF);
        int16_t curB = static_cast<int16_t>(val & 0x1FF);
        if (curA & 0x100)
            curA |= ~0x1FF;
        if (curB & 0x100)
            curB |= ~0x1FF;
        send_uart2_fmt("    CUR_A=%d CUR_B=%d\r\n",
            static_cast<int>(curA), static_cast<int>(curB));
    }
    if (read_and_print_reg(tmc, "  VACTUAL", tmc2208::REG_VACTUAL, val)) {
        int32_t vactual = static_cast<int32_t>(val);
        if (vactual & 0x00800000)
            vactual |= ~0x00FFFFFF;
        send_uart2_fmt("    VACTUAL=%ld\r\n", static_cast<long>(vactual));
    }
    if (read_and_print_reg(tmc, "  TPWMTHRS", tmc2208::REG_TPWMTHRS, val)) {
        (void)val;
    }

    send_uart2("\r\n", 2);
}

void log_tmc_frame(const char* tag, const uint8_t* data, size_t len)
{
    char line[96];
    int n = std::snprintf(
        line, sizeof(line), "%s[%lu]:", tag, static_cast<unsigned long>(len));
    if (n > 0) {
        send_uart2(line, static_cast<size_t>(n));
    }
    for (size_t i = 0; i < len; ++i) {
        n = std::snprintf(line, sizeof(line), " %02X", data[i]);
        if (n > 0) {
            send_uart2(line, static_cast<size_t>(n));
        }
    }
    send_uart2("\r\n", 2);
}

constexpr int32_t kVactualLimit = 0x007FFFFF; // ±23-bit range supported by VACTUAL

uint32_t encode_vactual(int32_t velocity)
{
    if (velocity > kVactualLimit) {
        velocity = kVactualLimit;
    } else if (velocity < -kVactualLimit) {
        velocity = -kVactualLimit;
    }
    return static_cast<uint32_t>(velocity) & 0x00FFFFFFu;
}

void set_velocity(tmc2208::Driver& tmc, int32_t velocity)
{
    tmc.write_reg(tmc2208::REG_VACTUAL, encode_vactual(velocity));
}

void run_motion_sequence(tmc2208::Driver& tmc)
{
    // Positive velocity -> forward.
    const int32_t kForwardSpeed = 2000; // microsteps per second
    const int32_t kReverseSpeed = -2000;

    set_velocity(tmc, kForwardSpeed);
    HAL_Delay(10000); // run forward for 10 s

    set_velocity(tmc, 0);
    HAL_Delay(2000); // pause for 2 s

    set_velocity(tmc, kReverseSpeed);
    HAL_Delay(6000); // run reverse for 6 s

    set_velocity(tmc, 0); // stop
}
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

    {
        const uint8_t probe_frame[] = { 0x05, 0x00, 0x00 };
        uint8_t crc_test = tmc2208::crc8_tmc(probe_frame, sizeof(probe_frame));
        char buf[32];
        int n = std::snprintf(
            buf, sizeof(buf), "CRC test: 0x%02X\r\n", crc_test);
        if (n > 0) {
            send_uart2(buf, static_cast<size_t>(n));
        }
    }

    auto tmc = tmc2208_hal::make_driver(&huart1);
    tmc2208_hal::set_logger(log_tmc_frame);
    tmc.set_address(kTmcSlaveAddress);
    HAL_Delay(50); // allow TMC startup and line settle

    bool tmc_ready = true;
    constexpr uint32_t kNodeconfSendDelay = (8u << 8); // SENDDELAY=8 byte times
    tmc.write_reg(tmc2208::REG_NODECONF, kNodeconfSendDelay);
    if (tmc.last_status() != tmc2208::Driver::Status::Ok) {
        report_tmc_status("write NODECONF", tmc.last_status());
        tmc_ready = false;
    }

    const uint32_t gconf = tmc2208::GCONF_PDN_DISABLE
        | tmc2208::GCONF_MSTEP_REG_SELECT;
    tmc.write_reg(tmc2208::REG_GCONF, gconf);
    if (tmc.last_status() != tmc2208::Driver::Status::Ok) {
        report_tmc_status("write GCONF", tmc.last_status());
        tmc_ready = false;
    }

    uint8_t microstep_code = 0;
    if (tmc_ready && !tmc2208::Driver::microsteps_to_code(16, microstep_code)) {
        report_tmc_status("microstep decode",
            tmc2208::Driver::Status::InvalidParam);
        tmc_ready = false;
    }
    if (tmc_ready && !tmc.set_microsteps_code(microstep_code)) {
        report_tmc_status("set microsteps", tmc.last_status());
        tmc_ready = false;
    }

    if (tmc_ready) {
        tmc.set_currents(10, 26, 4);
        if (tmc.last_status() != tmc2208::Driver::Status::Ok) {
            report_tmc_status("set currents", tmc.last_status());
            tmc_ready = false;
        }

        const uint32_t readback = tmc.read_reg(tmc2208::REG_GCONF);
        if (tmc.last_status() != tmc2208::Driver::Status::Ok) {
            report_tmc_status("read GCONF", tmc.last_status());
        } else {
            char msg[64];
            const int n = std::snprintf(
                msg, sizeof(msg), "TMC2208 GCONF=0x%08lX\r\n",
                static_cast<unsigned long>(readback));
            if (n > 0) {
                send_uart2(msg, static_cast<size_t>(n));
            }

            const uint32_t ioin = tmc.read_reg(tmc2208::REG_IOIN);
            if (tmc.last_status() != tmc2208::Driver::Status::Ok) {
                report_tmc_status("read IOIN", tmc.last_status());
            } else {
                const uint8_t version = static_cast<uint8_t>(ioin >> 24);
                char info[80];
                const int m = std::snprintf(info, sizeof(info),
                    "TMC2208 IOIN=0x%08lX version=0x%02X\r\n",
                    static_cast<unsigned long>(ioin), version);
                if (m > 0) {
                    send_uart2(info, static_cast<size_t>(m));
                }
            }
        }

        dump_tmc_diagnostics(tmc);
        run_motion_sequence(tmc);
        dump_tmc_diagnostics(tmc);
    }

    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        HAL_Delay(kBlinkIntervalMs);
    }
}
