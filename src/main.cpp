#include "board.hpp"
#include "command_processor.hpp"
#include "logger.hpp"
#include "main.h"
#include "motion_types.hpp"
#include "tmc5160_motor.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>

extern "C" {
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;
SPI_HandleTypeDef hspi1;
}

namespace {

constexpr uint32_t kBlinkIntervalMs = 1000;

struct ChipSelect {
    GPIO_TypeDef* port;
    uint16_t pin;
};

const ChipSelect kChipSelects[] = {
    { GPIOA, GPIO_PIN_0 },
    { GPIOA, GPIO_PIN_1 },
};

constexpr size_t kMotorCount = sizeof(kChipSelects) / sizeof(kChipSelects[0]);

struct MotorContext {
    tmc5160::Motor* driver = nullptr;
    tmc5160::Motor::Config cfg {};
    bool timed_move = false;
    uint32_t deadline_ms = 0;
    float last_velocity = 0.f;
    motion::Direction last_direction = motion::Direction::Forward;
};

MotorContext motors[kMotorCount];

char cmd_buffer[128];
uint8_t cmd_index = 0;
bool cmd_ready = false;

tmc5160::Motor::Config make_default_config()
{
    tmc5160::Motor::Config cfg;
    cfg.enable_spreadcycle = false;
    cfg.enable_stealthchop = true;
    cfg.use_internal_rsense = false;
    cfg.ihold = 16;
    cfg.irun = 28;
    cfg.ihold_delay = 6;
    cfg.tpowerdown = 20;
    cfg.microsteps = 256;
    cfg.toff = 4;
    cfg.hend = 1;
    cfg.hstrt = 4;
    cfg.blank_time = 2;
    cfg.high_vsense = false;
    cfg.enable_interpolation = true;
    cfg.double_edge_step = false;
    cfg.disable_s2g_protection = false;
    cfg.pwm_ampl = 128;
    cfg.pwm_grad = 4;
    cfg.pwm_freq = 1;
    cfg.pwm_autoscale = true;
    cfg.pwm_symmetric = false;
    cfg.pwm_freewheel = 0;
    cfg.write_tpwmthrs = false;
    cfg.write_tcoolthrs = false;
    cfg.write_thigh = false;
    cfg.write_global_scaler = false;
    cfg.write_dcctrl = false;
    cfg.write_a1 = false;
    cfg.write_v1 = false;
    cfg.write_d1 = false;
    cfg.vstart = 0;
    cfg.amax = 800;
    cfg.dmax = 800;
    cfg.vmax = 0;
    cfg.vstop = 10;
    cfg.tzerowait = 0;
    cfg.clock_frequency_hz = 12000000;
    return cfg;
}

tmc5160::Motor* acquire_motor(size_t index)
{
    switch (index) {
    case 0: {
        static SPI_Bridge bridge(&hspi1, kChipSelects[0].port, kChipSelects[0].pin);
        static tmc5160::Motor motor(bridge);
        return &motor;
    }
    case 1: {
        static SPI_Bridge bridge(&hspi1, kChipSelects[1].port, kChipSelects[1].pin);
        static tmc5160::Motor motor(bridge);
        return &motor;
    }
    default:
        return nullptr;
    }
}

tmc5160::Motor::Direction to_driver_direction(motion::Direction dir)
{
    return (dir == motion::Direction::Forward)
        ? tmc5160::Motor::Direction::Forward
        : tmc5160::Motor::Direction::Reverse;
}

float decode_velocity(uint32_t raw, uint32_t clock_hz)
{
    int32_t value = static_cast<int32_t>(raw & 0xFFFFFFu);
    if (value & 0x800000u)
        value |= ~0xFFFFFF;
    double velocity = static_cast<double>(value) * static_cast<double>(clock_hz)
        / static_cast<double>(1u << 24);
    return static_cast<float>(velocity);
}

void uart2_write(const char* msg)
{
    HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t*>(msg),
        static_cast<uint16_t>(std::strlen(msg)), HAL_MAX_DELAY);
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
    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& ctx = motors[i];
        if (!ctx.driver)
            continue;
        bool ok = fn(ctx, i);
        logsys::printf("[%s][M%u] %s\r\n", tag, static_cast<unsigned>(i),
            ok ? "OK" : "FAIL");
    }
}

uint16_t sanitize_microsteps(uint32_t requested)
{
    switch (requested) {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
    case 64:
    case 128:
    case 256:
        return static_cast<uint16_t>(requested);
    default:
        return 256;
    }
}

bool apply_velocity(MotorContext& ctx, float velocity_usteps_s, motion::Direction dir)
{
    if (!ctx.driver)
        return false;
    const float magnitude = std::fabs(velocity_usteps_s);
    if (!ctx.driver->setVelocity(magnitude, to_driver_direction(dir), ctx.cfg.clock_frequency_hz))
        return false;
    ctx.last_velocity = magnitude;
    ctx.last_direction = dir;
    return true;
}

void service_timed_moves()
{
    const uint32_t now = HAL_GetTick();
    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& ctx = motors[i];
        if (!ctx.driver || !ctx.timed_move)
            continue;
        const int32_t delta = static_cast<int32_t>(now - ctx.deadline_ms);
        if (delta >= 0) {
            if (ctx.driver->stop(ctx.cfg.clock_frequency_hz)) {
                logsys::printf("[STOP][M%u] timed run complete\r\n",
                    static_cast<unsigned>(i));
            }
            ctx.timed_move = false;
            ctx.last_velocity = 0.f;
        }
    }
}

void init_motor_contexts()
{
    for (size_t i = 0; i < kMotorCount; ++i) {
        MotorContext& ctx = motors[i];
        ctx.driver = acquire_motor(i);
        ctx.cfg = make_default_config();
        ctx.timed_move = false;
        ctx.deadline_ms = 0;
        ctx.last_velocity = 0.f;
        ctx.last_direction = motion::Direction::Forward;

        if (!ctx.driver) {
            logsys::printf("[SETUP][M%u] missing driver instance\r\n",
                static_cast<unsigned>(i));
            continue;
        }

        if (!ctx.driver->initialize(ctx.cfg)) {
            logsys::printf("[SETUP][M%u] SPI init failed\r\n",
                static_cast<unsigned>(i));
            continue;
        }

        uint32_t ioin = 0;
        if (ctx.driver->read(tmc5160::Reg::IOIN, ioin)) {
            const uint8_t version = static_cast<uint8_t>(ioin >> 24);
            logsys::printf("[IOIN][M%u] value=0x%08lX VERSION=0x%02X\r\n",
                static_cast<unsigned>(i),
                static_cast<unsigned long>(ioin),
                static_cast<unsigned>(version));
        } else {
            logsys::printf("[IOIN][M%u] read failed\r\n", static_cast<unsigned>(i));
        }
    }

    logsys::printf("[INIT] TMC5160 drivers ready.\r\n");
}

void print_status()
{
    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& ctx = motors[i];
        if (!ctx.driver)
            continue;

        uint32_t vactual = 0;
        bool have_vel = ctx.driver->read(tmc5160::Reg::VACTUAL, vactual);
        float velocity = have_vel ? decode_velocity(vactual, ctx.cfg.clock_frequency_hz) : 0.f;

        uint32_t drv_status = 0;
        bool have_drv = ctx.driver->read(tmc5160::Reg::DRV_STATUS, drv_status);
        const uint32_t sg_result = drv_status & tmc5160::DRV_STATUS_SG_RESULT_MASK;
        const uint32_t cs_actual = (drv_status & tmc5160::DRV_STATUS_CS_ACTUAL_MASK) >> 16;

        logsys::printf(
            "[STATUS][M%u] v=%.1f usteps/s dir=%s timed=%s drv=0x%08lX sg=%lu cs=%lu\r\n",
            static_cast<unsigned>(i), velocity,
            (ctx.last_direction == motion::Direction::Forward) ? "FWD" : "REV",
            ctx.timed_move ? "yes" : "no",
            have_drv ? static_cast<unsigned long>(drv_status) : 0ul,
            static_cast<unsigned long>(sg_result),
            static_cast<unsigned long>(cs_actual));
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
        logsys::printf("  speed <usteps_s> [dir]\r\n");
        logsys::printf("  run <usteps_s> [dir]\r\n");
        logsys::printf("  const <seconds> <usteps_s> [dir]\r\n");
        break;

    case command::Type::Status:
        print_status();
        break;

    case command::Type::Stop:
        for_each_motor("STOP", [](MotorContext& ctx, size_t) {
            if (!ctx.driver)
                return false;
            ctx.timed_move = false;
            ctx.last_velocity = 0.f;
            return ctx.driver->stop(ctx.cfg.clock_frequency_hz);
        });
        break;

    case command::Type::SetAmplitude: {
        const float clamped = std::clamp(cmd.amplitude, 0.f, 255.f);
        const float scale = clamped / 255.f;
        const int irun = static_cast<int>(std::lround(scale * 31.f));
        const int ihold = std::max(1, irun / 2);
        for_each_motor("CUR", [&](MotorContext& ctx, size_t) {
            if (!ctx.driver)
                return false;
            ctx.cfg.irun = static_cast<uint8_t>(irun);
            ctx.cfg.ihold = static_cast<uint8_t>(ihold);
            return ctx.driver->setCurrent(ctx.cfg.irun, ctx.cfg.ihold, ctx.cfg.ihold_delay);
        });
        break;
    }

    case command::Type::SetMicrosteps: {
        const uint16_t micro = sanitize_microsteps(cmd.microsteps);
        for_each_motor("MSTEPS", [&](MotorContext& ctx, size_t) {
            if (!ctx.driver)
                return false;
            ctx.cfg.microsteps = micro;
            return ctx.driver->setMicrosteps(micro, ctx.cfg);
        });
        break;
    }

    case command::Type::Run:
    case command::Type::Speed: {
        const float velocity = std::max(cmd.peak, 0.f);
        const motion::Direction dir = cmd.direction;
        for_each_motor("SPEED", [&](MotorContext& ctx, size_t) {
            if (!ctx.driver)
                return false;
            ctx.timed_move = false;
            return apply_velocity(ctx, velocity, dir);
        });
        break;
    }

    case command::Type::ConstantVelocity: {
        const float velocity = std::max(cmd.peak, 0.f);
        const motion::Direction dir = cmd.direction;
        const uint32_t duration_ms
            = static_cast<uint32_t>((cmd.duration > 0.f ? cmd.duration : 0.f) * 1000.f);
        for_each_motor("CONST", [&](MotorContext& ctx, size_t) {
            if (!ctx.driver)
                return false;
            if (!apply_velocity(ctx, velocity, dir))
                return false;
            if (duration_ms > 0u) {
                ctx.timed_move = true;
                ctx.deadline_ms = HAL_GetTick() + duration_ms;
            } else {
                ctx.timed_move = false;
            }
            return true;
        });
        break;
    }

    case command::Type::Scurve:
    case command::Type::Triangle:
    case command::Type::Trapezoid:
    case command::Type::Expo:
    case command::Type::Sine:
        logsys::printf("[CMD] Profile commands are not supported in SPI velocity mode.\r\n");
        break;

    case command::Type::None:
    default:
        logsys::printf("[CMD] Unknown. Type 'help'.\r\n");
        break;
    }
}

void print_banner()
{
    uart2_write("\r\n=== TMC5160 SPI Demo ===\r\n");
}

} // namespace

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    BlinkyLED();

    logsys::init(&huart2);
    print_banner();
    logsys::printf("[BOOT] Ready.\r\n");

    TMC_DriversEnable(true);
    HAL_Delay(2);
    init_motor_contexts();

    uint32_t last_blink = 0;

    while (true) {
        handle_uart_input();
        process_command();
        service_timed_moves();

        const uint32_t now = HAL_GetTick();
        if (now - last_blink >= kBlinkIntervalMs) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            last_blink = now;
        }

        HAL_Delay(1);
    }
}
