#include "board.hpp"
#include "logger.hpp"
#include "main.h"
#include "motion_profile.hpp"
#include "spi_bridge.hpp"
#include "tmc2130_motor.hpp"
#include "command_processor.hpp"
#include <cstring>

extern "C" {
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;
}

namespace {

constexpr uint32_t kBlinkIntervalMs = 1000;

struct MotorContext {
    SPI_Bridge* spi = nullptr;
    tmc2130::Motor* driver = nullptr;
    motion::MotionProfile* profile = nullptr;
};

struct CsPin {
    GPIO_TypeDef* port;
    uint16_t pin;
};

const CsPin kCsPins[] = {
    { GPIOA, GPIO_PIN_4 },
    { GPIOB, GPIO_PIN_0 },
};

MotorContext motors[2];

static_assert(sizeof(kCsPins) / sizeof(kCsPins[0]) == sizeof(motors) / sizeof(motors[0]),
    "Mismatch between CS pin table and motor context count");

char cmd_buffer[128];
uint8_t cmd_index = 0;
bool cmd_ready = false;

void uart2_write(const char* msg)
{
    HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t*>(msg),
        static_cast<uint16_t>(strlen(msg)), HAL_MAX_DELAY);
}

void handle_uart_input()
{
    uint8_t byte = 0;
    if (HAL_UART_Receive(&huart2, &byte, 1, 0) != HAL_OK)
        return;

    if (byte == '\r' || byte == '\n') {
        if (cmd_index > 0) {
            cmd_buffer[cmd_index] = '\0';
            cmd_ready = true;
        }
        cmd_index = 0;
        return;
    }

    if (byte >= 32 && byte <= 126 && cmd_index < sizeof(cmd_buffer) - 1) {
        cmd_buffer[cmd_index++] = static_cast<char>(byte);
        HAL_UART_Transmit(&huart2, &byte, 1, HAL_MAX_DELAY);
    }
}

template <typename Fn>
void for_each_motor(const char* tag, Fn&& fn)
{
    for (size_t i = 0; i < 2; ++i) {
        auto* profile = motors[i].profile;
        if (!profile)
            continue;
        bool ok = fn(*profile);
        logsys::printf("[%s][M%u] %s\r\n", tag, static_cast<unsigned>(i),
            ok ? "OK" : "FAIL");
    }
}

void configure_cs_pins()
{
    for (const auto& cs : kCsPins) {
        GPIO_InitTypeDef gpio = { 0 };
        gpio.Pin = cs.pin;
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(cs.port, &gpio);
        HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_SET);
    }
}

void init_motor_contexts()
{
    static SPI_Bridge spi0(&hspi1, kCsPins[0].port, kCsPins[0].pin);
    static SPI_Bridge spi1(&hspi1, kCsPins[1].port, kCsPins[1].pin);
    static tmc2130::Motor drv0(spi0);
    static tmc2130::Motor drv1(spi1);
    static motion::MotionProfile prof0(drv0, &htim2);
    static motion::MotionProfile prof1(drv1, &htim2);

    motors[0] = MotorContext { &spi0, &drv0, &prof0 };
    motors[1] = MotorContext { &spi1, &drv1, &prof1 };

    for_each_motor("SETUP", [](motion::MotionProfile& p) {
        p.setAmplitude(200.f);
        p.setMicrostepResolution(256);
        return true;
    });

    tmc2130::Motor::Config cfg;
    cfg.direct_mode = true;
    cfg.enable_stealthchop = false;
    cfg.write_pwmconf = false;
    cfg.write_chopconf = true;

    for (auto& ctx : motors) {
        if (ctx.driver && ctx.driver->initialize(cfg)) {
            logsys::printf("[INIT] motor ready\r\n");
        } else {
            logsys::printf("[INIT] motor init failed\r\n");
        }
    }
}

void process_command()
{
    if (!cmd_ready)
        return;

    cmd_ready = false;
    logsys::printf("\r\n[CMD] %s\r\n", cmd_buffer);

    command::Command cmd = command::parse(cmd_buffer);

    switch (cmd.type) {
    case command::Type::Help:
        logsys::printf("Commands:\r\n");
        logsys::printf("  help\r\n");
        logsys::printf("  status\r\n");
        logsys::printf("  stop\r\n");
        logsys::printf("  amplitude <0..255>\r\n");
        logsys::printf("  microsteps <steps>\r\n");
        logsys::printf("  scurve <s> <usteps> [dir] [period_ms]\r\n");
        logsys::printf("  triangle <s> <usteps> [dir] [period_ms]\r\n");
        logsys::printf(
            "  trapezoid <accel> <const> <decel> <usteps> [dir] [period_ms]\r\n");
        logsys::printf("  expo <s> <usteps> [steep] [dir] [period_ms]\r\n");
        logsys::printf("  sine <s> <usteps> [dir] [period_ms]\r\n");
        logsys::printf("  const <s> <usteps> [dir]\r\n");
        break;
    case command::Type::Status:
        for (size_t i = 0; i < 2; ++i) {
            auto* drv = motors[i].driver;
            if (!drv)
                continue;
            uint32_t value = 0;
            if (drv->read(tmc2130::Reg::DRV_STATUS, value)) {
                logsys::printf("[STATUS][M%u] 0x%08lX (SPI=0x%02X)\r\n",
                    static_cast<unsigned>(i), value,
                    static_cast<unsigned>(drv->lastStatus()));
            } else {
                logsys::printf("[STATUS][M%u] read failed\r\n",
                    static_cast<unsigned>(i));
            }
        }
        break;
    case command::Type::Stop:
        for_each_motor("STOP", [](motion::MotionProfile& p) {
            return p.stop();
        });
        break;
    case command::Type::SetAmplitude: {
        float amp = (cmd.amplitude < 0.f) ? 0.f : cmd.amplitude;
        for_each_motor("AMP", [amp](motion::MotionProfile& p) {
            p.setAmplitude(amp);
            return true;
        });
        break;
    }
    case command::Type::SetMicrosteps: {
        uint32_t ms = cmd.microsteps ? cmd.microsteps : 256u;
        for_each_motor("MSTEPS", [ms](motion::MotionProfile& p) {
            p.setMicrostepResolution(static_cast<uint16_t>(ms));
            return true;
        });
        break;
    }
    case command::Type::Scurve:
        for_each_motor("SCURVE", [&](motion::MotionProfile& p) {
            return p.runSCurve(cmd.duration, cmd.peak, cmd.direction,
                cmd.period_ms ? cmd.period_ms : 10);
        });
        break;
    case command::Type::Triangle:
        for_each_motor("TRIANGLE", [&](motion::MotionProfile& p) {
            return p.runTriangular(cmd.duration, cmd.peak, cmd.direction,
                cmd.period_ms ? cmd.period_ms : 10);
        });
        break;
    case command::Type::Trapezoid:
        for_each_motor("TRAP", [&](motion::MotionProfile& p) {
            return p.runTrapezoidal(cmd.accel, cmd.constant, cmd.decel, cmd.peak,
                cmd.direction, cmd.period_ms ? cmd.period_ms : 10);
        });
        break;
    case command::Type::Expo:
        for_each_motor("EXPO", [&](motion::MotionProfile& p) {
            return p.runExponential(cmd.duration, cmd.peak, cmd.steep, cmd.direction,
                cmd.period_ms ? cmd.period_ms : 10);
        });
        break;
    case command::Type::Sine:
        for_each_motor("SINE", [&](motion::MotionProfile& p) {
            return p.runSinusoidal(cmd.duration, cmd.peak, cmd.direction,
                cmd.period_ms ? cmd.period_ms : 10);
        });
        break;
    case command::Type::ConstantVelocity:
        for_each_motor("CONST", [&](motion::MotionProfile& p) {
            return p.runConstantVelocity(cmd.duration, cmd.peak, cmd.direction);
        });
        break;
    case command::Type::None:
    default:
        logsys::printf("[CMD] Unknown. Type 'help'.\r\n");
        break;
    }
}

} // namespace

static void print_banner()
{
    uart2_write("\r\n=== TMC2130 SPI Demo ===\r\n");
}

int main()
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

    logsys::init(&huart2);
    print_banner();
    logsys::printf("[BOOT] Ready.\r\n");

    configure_cs_pins();
    init_motor_contexts();

    while (true) {
        handle_uart_input();
        process_command();

        static uint32_t last_blink = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_blink >= kBlinkIntervalMs) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
            last_blink = now;
        }

        HAL_Delay(1);
    }
}
