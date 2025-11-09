#include "main.h"
#include "board.hpp"
#include "command_processor.hpp"
#include "logger.hpp"
#include "motion_types.hpp"
#include "tmc5160_motor.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>

using std::lround;
using std::max;
using std::min;

extern "C" {
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;
SPI_HandleTypeDef hspi1;
}

namespace
{

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
    cfg.enable_pwm_autoscale = true;
    cfg.enable_pwm_autograd = false;
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
    cfg.amax = 5000;
    cfg.dmax = 5000;
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
        static SPI_Bridge bridge(
            &hspi1, kChipSelects[0].port, kChipSelects[0].pin);
        static tmc5160::Motor motor(bridge);
        return &motor;
    }
    case 1: {
        static SPI_Bridge bridge(
            &hspi1, kChipSelects[1].port, kChipSelects[1].pin);
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

template <typename Fn> void for_each_motor(const char* tag, Fn&& fn)
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

bool apply_velocity(
    MotorContext& ctx, float velocity_usteps_s, motion::Direction dir)
{
    if (!ctx.driver)
        return false;
    const float magnitude = std::fabs(velocity_usteps_s);

    // Calculate expected VMAX register value for debugging
    double expected_vmax = static_cast<double>(magnitude) * (1ull << 24)
        / static_cast<double>(ctx.cfg.clock_frequency_hz);
    uint32_t vmax_reg = static_cast<uint32_t>(expected_vmax);

    logsys::printf(
        "[DEBUG] setVelocity: vel=%.0f clock=%lu vmax_reg=0x%08lX\r\n",
        magnitude, (unsigned long)ctx.cfg.clock_frequency_hz,
        (unsigned long)vmax_reg);

    if (!ctx.driver->setVelocity(
            magnitude, to_driver_direction(dir), ctx.cfg.clock_frequency_hz))
        return false;

    // Read back VMAX to verify
    uint32_t readback = ctx.driver->read(tmc5160::Reg::VMAX);
    logsys::printf(
        "[DEBUG] VMAX readback=0x%08lX\r\n", (unsigned long)readback);

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

        // Debug print register addresses and raw values
        uint8_t gconf_addr = static_cast<uint8_t>(tmc5160::Reg::GCONF);
        uint8_t ioin_addr = static_cast<uint8_t>(tmc5160::Reg::IOIN);
        logsys::printf("[DEBUG][M%u] GCONF=0x%02X IOIN=0x%02X\r\n",
            static_cast<unsigned>(i), gconf_addr, ioin_addr);

        // Try to read GCONF first to verify basic SPI communication
        uint32_t gconf = 0;
        if (!ctx.driver->read(tmc5160::Reg::GCONF, gconf)) {
            logsys::printf("[SETUP][M%u] Initial GCONF read failed - Check SPI "
                           "connections\r\n",
                static_cast<unsigned>(i));
            continue;
        }

        logsys::printf("[SETUP][M%u] GCONF=0x%08lX\r\n",
            static_cast<unsigned>(i), static_cast<unsigned long>(gconf));

        // Now try to initialize the driver
        if (!ctx.driver->initialize(ctx.cfg)) {
            logsys::printf("[SETUP][M%u] Driver initialization failed\r\n",
                static_cast<unsigned>(i));
            continue;
        }

        // Read and validate IOIN register
        uint32_t ioin = 0;
        bool init_ok = false;

        if (ctx.driver->read(tmc5160::Reg::IOIN, ioin)) {
            const uint8_t version
                = static_cast<uint8_t>(tmc5160::IOIN::VERSION.get(ioin));
            const bool en = tmc5160::IOIN::DRV_ENN.get(ioin);

            logsys::printf("[IOIN][M%u] value=0x%08lX VERSION=0x%02X %s %s\r\n",
                static_cast<unsigned>(i), static_cast<unsigned long>(ioin),
                static_cast<unsigned>(version),
                (version == 0x30) ? "[VERSION OK]"
                                  : "[BAD VERSION - Expected 0x30]",
                !en ? "[POWER OK]"
                    : "[NO POWER - Check VMO is its switched on]");

            // Only continue if version is correct and power is present
            if (version == 0x30) {
                if (!en) {
                    init_ok = true;
                } else {
                    logsys::printf(
                        "[INIT][M%u] Driver power not detected (DRV_ENN=1)\r\n",
                        static_cast<unsigned>(i));
                }
            } else {
                logsys::printf(
                    "[INIT][M%u] Invalid version 0x%02X (expected 0x30)\r\n",
                    static_cast<unsigned>(i), static_cast<unsigned>(version));
            }
        } else {
            logsys::printf(
                "[IOIN][M%u] read failed - Check SPI connections\r\n",
                static_cast<unsigned>(i));
        }

        // If initialization failed, clear the driver pointer
        if (!init_ok) {
            ctx.driver = nullptr;
        }
    }

    // Check if any drivers initialized successfully
    size_t ready_count = 0;
    for (size_t i = 0; i < kMotorCount; ++i) {
        if (motors[i].driver != nullptr) {
            ++ready_count;
        }
    }

    const char* status = "failed to initialize";
    if (ready_count == kMotorCount) {
        status = "ready";
    } else if (ready_count > 0) {
        status = "partially ready";
    }

    logsys::printf("[INIT] TMC5160 drivers %s (%u/%u)\r\n", status,
        static_cast<unsigned>(ready_count), static_cast<unsigned>(kMotorCount));
}

void print_status()
{
    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& ctx = motors[i];
        if (!ctx.driver)
            continue;

        // Read registers
        uint32_t vactual = 0;
        uint32_t drv_status = 0;
        uint32_t ioin = 0;

        ctx.driver->read(tmc5160::Reg::VACTUAL, vactual);
        ctx.driver->read(tmc5160::Reg::DRV_STATUS, drv_status);
        ctx.driver->read(tmc5160::Reg::IOIN, ioin);

        // Process readings
        float velocity = decode_velocity(vactual, ctx.cfg.clock_frequency_hz);
        const long velocity_int = lround(velocity);
        uint32_t sg_result = tmc5160::DRV_STATUS::SG_RESULT.get(drv_status);
        uint32_t cs_actual = tmc5160::DRV_STATUS::CS_ACTUAL.get(drv_status);

        // Print status
        const char* dir_str = (ctx.last_direction == motion::Direction::Forward)
            ? "FWD"
            : "REV";
        const char* timed_str = ctx.timed_move ? "yes" : "no";

        logsys::printf("[STATUS][M%u] v=%ld usteps/s dir=%s timed=%s\r\n",
            static_cast<unsigned>(i), velocity_int, dir_str, timed_str);

        logsys::printf("    DRV: 0x%08lX sg=%lu cs=%lu\r\n",
            (unsigned long)drv_status, (unsigned long)sg_result,
            (unsigned long)cs_actual);

        // Print IO pin states with validation
        const auto ref_step = tmc5160::IOIN::REFL_STEP.get(ioin);
        const auto ref_dir = tmc5160::IOIN::REFR_DIR.get(ioin);
        const auto step_pin = tmc5160::IOIN::STEP.get(ioin);
        const auto dir_pin = tmc5160::IOIN::DIR.get(ioin);
        const auto en = tmc5160::IOIN::DRV_ENN.get(ioin);
        const auto mode = tmc5160::IOIN::SD_MODE.get(ioin);
        const auto version = tmc5160::IOIN::VERSION.get(ioin);

        // Expected version for TMC5160 is 0x30
        bool version_ok = (version == 0x30);
        bool power_ok
            = !en; // DRV_ENN is active low, so !en means power is good

        const bool step_dir_enabled = (mode == 0);
        if (step_dir_enabled) {
            logsys::printf(
                "    IO:  STEP_IN=%lu DIR_IN=%lu REF_STEP=%lu REF_DIR=%lu "
                "EN=%lu MODE=%lu [STEP/DIR] VER=0x%02lX  %s %s\r\n",
                (unsigned long)step_pin, (unsigned long)dir_pin,
                (unsigned long)ref_step, (unsigned long)ref_dir,
                (unsigned long)en, (unsigned long)mode, (unsigned long)version,
                version_ok ? "[VERSION OK]" : "[BAD VERSION - Expected 0x30]",
                power_ok ? "[POWER OK]" : "[NO POWER - Check 12V]");
        } else {
            logsys::printf(
                "    IO:  REF_STEP=%lu REF_DIR=%lu EN=%lu MODE=%lu [SPI] "
                "VER=0x%02lX  %s %s\r\n",
                (unsigned long)ref_step, (unsigned long)ref_dir,
                (unsigned long)en, (unsigned long)mode, (unsigned long)version,
                version_ok ? "[VERSION OK]" : "[BAD VERSION - Expected 0x30]",
                power_ok ? "[POWER OK]" : "[NO POWER - Check 12V]");
            logsys::printf("         STEP/DIR interface disabled (velocity "
                           "mode active)\r\n");
        }
    }
}

void log_drv_status(const char* tag)
{
    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& ctx = motors[i];
        if (!ctx.driver)
            continue;
        uint32_t drv_status = 0;
        if (ctx.driver->read(tmc5160::Reg::DRV_STATUS, drv_status)) {
            const auto sg = tmc5160::DRV_STATUS::SG_RESULT.get(drv_status);
            const auto cs = tmc5160::DRV_STATUS::CS_ACTUAL.get(drv_status);
            const auto stealth = tmc5160::DRV_STATUS::STEALTH.get(drv_status);
            const auto fsactive = tmc5160::DRV_STATUS::FSACTIVE.get(drv_status);
            const auto stst = tmc5160::DRV_STATUS::STST.get(drv_status);
            const auto stall_guard
                = tmc5160::DRV_STATUS::STALLGUARD.get(drv_status);
            const auto ot = tmc5160::DRV_STATUS::OT.get(drv_status);
            const auto otpw = tmc5160::DRV_STATUS::OTPW.get(drv_status);
            const auto s2ga = tmc5160::DRV_STATUS::S2GA.get(drv_status);
            const auto s2gb = tmc5160::DRV_STATUS::S2GB.get(drv_status);
            const auto ola = tmc5160::DRV_STATUS::OLA.get(drv_status);
            const auto olb = tmc5160::DRV_STATUS::OLB.get(drv_status);
            const auto s2vsa = tmc5160::DRV_STATUS::S2VSA.get(drv_status);
            const auto s2vsb = tmc5160::DRV_STATUS::S2VSB.get(drv_status);
            logsys::printf(
                "[%s][M%u] raw=0x%08lX sg=%lu cs=%lu stealth=%lu fs=%lu "
                "stst=%lu stall=%lu ot=%lu otpw=%lu s2ga=%lu s2gb=%lu "
                "ola=%lu olb=%lu s2vsa=%lu s2vsb=%lu\r\n",
                tag, static_cast<unsigned>(i), (unsigned long)drv_status,
                (unsigned long)sg, (unsigned long)cs, (unsigned long)stealth,
                (unsigned long)fsactive, (unsigned long)stst,
                (unsigned long)stall_guard, (unsigned long)ot,
                (unsigned long)otpw, (unsigned long)s2ga, (unsigned long)s2gb,
                (unsigned long)ola, (unsigned long)olb, (unsigned long)s2vsa,
                (unsigned long)s2vsb);
        } else {
            logsys::printf("[%s][M%u] DRV_STATUS read failed\r\n", tag,
                static_cast<unsigned>(i));
        }
    }
}

void log_motion_state(const char* tag)
{
    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& ctx = motors[i];
        if (!ctx.driver)
            continue;

        uint32_t ramp_mode = 0;
        const bool mode_ok
            = ctx.driver->read(tmc5160::Reg::RAMPMODE, ramp_mode);
        const char* mode_str = "UNKNOWN";
        if (mode_ok) {
            switch (ramp_mode & 0x3u) {
            case 0:
                mode_str = "POSITION";
                break;
            case 1:
                mode_str = "VEL+";
                break;
            case 2:
                mode_str = "VEL-";
                break;
            case 3:
                mode_str = "HOLD";
                break;
            default:
                break;
            }
        }

        uint32_t ramp_stat = 0;
        if (!ctx.driver->read(tmc5160::Reg::RAMP_STAT, ramp_stat)) {
            logsys::printf(
                "[%s][M%u] RAMPMODE=%s (raw=0x%08lX) RAMP_STAT read failed\r\n",
                tag, static_cast<unsigned>(i), mode_str,
                mode_ok ? (unsigned long)ramp_mode : 0ul);
            continue;
        }

        const auto stop_l = tmc5160::RAMP_STAT::STATUS_STOP_L.get(ramp_stat);
        const auto stop_r = tmc5160::RAMP_STAT::STATUS_STOP_R.get(ramp_stat);
        const auto latch_l = tmc5160::RAMP_STAT::STATUS_LATCH_L.get(ramp_stat);
        const auto latch_r = tmc5160::RAMP_STAT::STATUS_LATCH_R.get(ramp_stat);
        const auto evt_stop_l = tmc5160::RAMP_STAT::EVENT_STOP_L.get(ramp_stat);
        const auto evt_stop_r = tmc5160::RAMP_STAT::EVENT_STOP_R.get(ramp_stat);
        const auto evt_stop_sg
            = tmc5160::RAMP_STAT::EVENT_STOP_SG.get(ramp_stat);
        const auto evt_pos
            = tmc5160::RAMP_STAT::EVENT_POS_REACHED.get(ramp_stat);
        const auto vel_reached
            = tmc5160::RAMP_STAT::VELOCITY_REACHED.get(ramp_stat);
        const auto pos_reached
            = tmc5160::RAMP_STAT::POSITION_REACHED.get(ramp_stat);
        const auto vzero = tmc5160::RAMP_STAT::VZERO.get(ramp_stat);

        uint32_t vactual = 0;
        bool vactual_ok = ctx.driver->read(tmc5160::Reg::VACTUAL, vactual);
        const char* vact_str = "?";
        char vact_buf[16];
        if (vactual_ok) {
            const float vel
                = decode_velocity(vactual, ctx.cfg.clock_frequency_hz);
            const long vactual_usteps = lround(vel);
            snprintf(vact_buf, sizeof(vact_buf), "%ld", vactual_usteps);
            vact_str = vact_buf;
        }

        uint32_t vmax_reg = 0;
        bool vmax_ok = ctx.driver->read(tmc5160::Reg::VMAX, vmax_reg);
        const char* vmax_str = "?";
        char vmax_buf[32];
        if (vmax_ok) {
            const float vmax_vel
                = decode_velocity(vmax_reg, ctx.cfg.clock_frequency_hz);
            const long vmax_usteps = lround(vmax_vel);
            snprintf(vmax_buf, sizeof(vmax_buf), "%ld (0x%lX)", vmax_usteps,
                (unsigned long)vmax_reg);
            vmax_str = vmax_buf;
        }

        logsys::printf("[%s][M%u] RAMPMODE=%s (0x%08lX) RAMP_STAT=0x%08lX "
                       "stopL=%lu stopR=%lu latchL=%lu latchR=%lu "
                       "eventL=%lu eventR=%lu eventSG=%lu posEvt=%lu vel=%lu "
                       "pos=%lu vzero=%lu vact=%s vmax=%s\r\n",
            tag, static_cast<unsigned>(i), mode_str,
            mode_ok ? (unsigned long)ramp_mode : 0ul, (unsigned long)ramp_stat,
            (unsigned long)stop_l, (unsigned long)stop_r,
            (unsigned long)latch_l, (unsigned long)latch_r,
            (unsigned long)evt_stop_l, (unsigned long)evt_stop_r,
            (unsigned long)evt_stop_sg, (unsigned long)evt_pos,
            (unsigned long)vel_reached, (unsigned long)pos_reached,
            (unsigned long)vzero, vact_str, vmax_str);
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
        const float clamped = (cmd.amplitude < 0.f)
            ? 0.f
            : (cmd.amplitude > 255.f ? 255.f : cmd.amplitude);
        const float scale = clamped / 255.f;
        const int irun = static_cast<int>(scale * 31.f + 0.5f);
        const int ihold = (irun / 2 < 1) ? 1 : (irun / 2);
        for_each_motor("CUR", [&](MotorContext& ctx, size_t) {
            if (!ctx.driver)
                return false;
            ctx.cfg.irun = static_cast<uint8_t>(irun);
            ctx.cfg.ihold = static_cast<uint8_t>(ihold);
            return ctx.driver->setCurrent(
                ctx.cfg.irun, ctx.cfg.ihold, ctx.cfg.ihold_delay);
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
        const float velocity = (cmd.peak < 0.f) ? 0.f : cmd.peak;
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
        const float velocity = (cmd.peak < 0.f) ? 0.f : cmd.peak;
        const motion::Direction dir = cmd.direction;
        const uint32_t duration_ms = static_cast<uint32_t>(
            (cmd.duration > 0.f ? cmd.duration : 0.f) * 1000.f);
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

    case command::Type::None:
    default:
        logsys::printf("[CMD] Unknown. Type 'help'.\r\n");
        break;
    }

    log_drv_status("DRV");
    log_motion_state("RAMP");
}

void print_banner() { uart2_write("\r\n=== TMC5160 SPI Demo ===\r\n"); }

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

    // Enable drivers and wait for power-up
    TMC_DriversEnable(true);
    HAL_Delay(500); // Increase power-up delay to 500ms

    // Configure SPI at lower speed initially
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }

    init_motor_contexts();

    // After init, can increase SPI speed if needed
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }

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
