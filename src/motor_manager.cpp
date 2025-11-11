#include "motor_manager.hpp"
#include "board.hpp"
#include "logger.hpp"
#include "spi_bridge.hpp"
#include "tmc5160_motor.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

extern "C" {
extern SPI_HandleTypeDef hspi1;
}

namespace motor_manager
{

namespace
{

    // Chip select pins for each motor
    struct ChipSelect {
        GPIO_TypeDef* port;
        uint16_t pin;
    };

    const ChipSelect kChipSelects[] = {
        { GPIOA, GPIO_PIN_0 },
        { GPIOA, GPIO_PIN_1 },
    };

    constexpr size_t kMotorCount
        = sizeof(kChipSelects) / sizeof(kChipSelects[0]);

    // Array of motor contexts
    MotorContext motors[kMotorCount];

    // Create default TMC5160 configuration
    tmc5160::Motor::Config make_default_config()
    {
        tmc5160::Motor::Config cfg;

        // Driver mode settings
        cfg.enable_spreadcycle = false; // Use stealthChop (quieter)
        cfg.enable_stealthchop = true;
        cfg.use_internal_rsense = false; // External sense resistors

        // Current settings (out of 31 max)
        cfg.ihold = 16; // Holding current
        cfg.irun = 28; // Running current
        cfg.ihold_delay = 6; // Delay before reducing to hold current
        cfg.tpowerdown = 20; // Power down delay

        // Microstepping and chopper settings
        cfg.microsteps = 256; // 256 microsteps per full step
        cfg.toff = 4; // Chopper off time
        cfg.hend = 1; // Hysteresis end value
        cfg.hstrt = 4; // Hysteresis start value
        cfg.blank_time = 2; // Comparator blank time
        cfg.high_vsense = false; // Low sense resistor voltage
        cfg.enable_interpolation
            = true; // Enable step interpolation to 256 µsteps
        cfg.double_edge_step = false;
        cfg.disable_s2g_protection = false;

        // StealthChop PWM settings
        cfg.pwm_ampl = 128; // PWM amplitude (autoscale will adjust)
        cfg.pwm_grad = 4; // PWM gradient
        cfg.pwm_freq = 1; // PWM frequency
        cfg.enable_pwm_autoscale = true; // Enable automatic PWM scaling
        cfg.enable_pwm_autograd = false;
        cfg.pwm_freewheel = 0;

        // Optional threshold registers (not used)
        cfg.write_tpwmthrs = false;
        cfg.write_tcoolthrs = false;
        cfg.write_thigh = false;
        cfg.write_global_scaler = false;
        cfg.write_dcctrl = false;
        cfg.write_a1 = false;
        cfg.write_v1 = false;
        cfg.write_d1 = false;

        // Ramp generator parameters
        cfg.vstart = 0; // Start velocity (0 = start from standstill)
        cfg.amax = 5000; // Acceleration (increased for faster ramp-up)
        cfg.dmax = 5000; // Deceleration
        cfg.vmax = 0; // Max velocity (set by commands)
        cfg.vstop = 10; // Stop velocity
        cfg.tzerowait = 0; // Zero wait time

        // Clock frequency for velocity calculations
        cfg.clock_frequency_hz = 12000000; // 12 MHz internal clock

        return cfg;
    }

    // Acquire motor driver instance (static instances)
    tmc5160::Motor* acquire_motor(size_t index)
    {
        switch (index) {
        case 0: {
            // Motor 0: Static SPI bridge and driver
            static SPI_Bridge bridge(
                &hspi1, kChipSelects[0].port, kChipSelects[0].pin);
            static tmc5160::Motor motor(bridge);
            return &motor;
        }
        case 1: {
            // Motor 1: Static SPI bridge and driver
            static SPI_Bridge bridge(
                &hspi1, kChipSelects[1].port, kChipSelects[1].pin);
            static tmc5160::Motor motor(bridge);
            return &motor;
        }
        default:
            return nullptr; // Invalid index
        }
    }

    // Convert motion::Direction to tmc5160::Motor::Direction
    tmc5160::Motor::Direction to_driver_direction(motion::Direction dir)
    {
        return (dir == motion::Direction::Forward)
            ? tmc5160::Motor::Direction::Forward
            : tmc5160::Motor::Direction::Reverse;
    }

    // Decode VACTUAL register (signed 24-bit) to microsteps/second
    float decode_velocity(uint32_t raw, uint32_t clock_hz)
    {
        // Extract 24-bit value and sign-extend if negative
        int32_t value = static_cast<int32_t>(raw & 0xFFFFFFu);
        if (value & 0x800000u) // If sign bit is set
            value |= ~0xFFFFFF; // Sign-extend to 32 bits

        // Convert to microsteps/second using TMC5160 formula
        double velocity = static_cast<double>(value)
            * static_cast<double>(clock_hz) / static_cast<double>(1u << 24);
        return static_cast<float>(velocity);
    }

} // anonymous namespace

// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

void initialize()
{
    // Initialize each motor context
    for (size_t i = 0; i < kMotorCount; ++i) {
        MotorContext& ctx = motors[i];

        // Acquire driver instance and set default config
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

        // Debug: Print register addresses
        uint8_t gconf_addr = static_cast<uint8_t>(tmc5160::Reg::GCONF);
        uint8_t ioin_addr = static_cast<uint8_t>(tmc5160::Reg::IOIN);
        logsys::printf("[DEBUG][M%u] GCONF=0x%02X IOIN=0x%02X\r\n",
            static_cast<unsigned>(i), gconf_addr, ioin_addr);

        // Verify SPI communication by reading GCONF
        uint32_t gconf = 0;
        if (!ctx.driver->read(tmc5160::Reg::GCONF, gconf)) {
            logsys::printf("[SETUP][M%u] Initial GCONF read failed - Check SPI "
                           "connections\r\n",
                static_cast<unsigned>(i));
            continue;
        }

        logsys::printf("[SETUP][M%u] GCONF=0x%08lX\r\n",
            static_cast<unsigned>(i), static_cast<unsigned long>(gconf));

        // Initialize driver with default configuration
        if (!ctx.driver->initialize(ctx.cfg)) {
            logsys::printf("[SETUP][M%u] Driver initialization failed\r\n",
                static_cast<unsigned>(i));
            continue;
        }

        // Read and validate IOIN register (version and power status)
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
                !en ? "[POWER OK]" : "[NO POWER - Check VM is switched on]");

            // Only mark as initialized if version is correct and power is
            // present
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

    // Count successfully initialized drivers
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

MotorContext* get_context(size_t index)
{
    if (index >= kMotorCount)
        return nullptr;
    return &motors[index];
}

size_t get_motor_count() { return kMotorCount; }

bool apply_velocity(
    MotorContext& ctx, float velocity_usteps_s, motion::Direction dir)
{
    if (!ctx.driver)
        return false;

    const float magnitude
        = std::fabs(velocity_usteps_s); // Ensure positive velocity

    // Call TMC5160 setVelocity (sets RAMPMODE and VMAX)
    if (!ctx.driver->setVelocity(
            magnitude, to_driver_direction(dir), ctx.cfg.clock_frequency_hz))
        return false;

    // Update context tracking
    ctx.last_velocity = magnitude;
    ctx.last_direction = dir;
    return true;
}

void service_timed_moves()
{
    const uint32_t now
        = HAL_GetTick(); // Get current system time in milliseconds

    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& ctx = motors[i];

        // Skip if no driver or no active timed move
        if (!ctx.driver || !ctx.timed_move)
            continue;

        // Check if deadline has passed
        const int32_t delta = static_cast<int32_t>(now - ctx.deadline_ms);
        if (delta >= 0) {
            // Time's up! Stop the motor
            if (ctx.driver->stop(ctx.cfg.clock_frequency_hz)) {
                logsys::printf("[STOP][M%u] timed run complete\r\n",
                    static_cast<unsigned>(i));
            }
            ctx.timed_move = false;
            ctx.last_velocity = 0.f;
        }
    }
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
        const long velocity_int = std::lround(velocity);
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

        bool version_ok = (version == 0x30);
        bool power_ok = !en; // DRV_ENN is active low

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
            logsys::printf("    IO:  REF_STEP=%lu REF_DIR=%lu EN=%lu MODE=%lu "
                           "[SPI] VER=0x%02lX  %s %s\r\n",
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

            logsys::printf("[%s][M%u] raw=0x%08lX sg=%lu cs=%lu stealth=%lu "
                           "fs=%lu stst=%lu stall=%lu "
                           "ot=%lu otpw=%lu s2ga=%lu s2gb=%lu ola=%lu olb=%lu "
                           "s2vsa=%lu s2vsb=%lu\r\n",
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

        // Read RAMPMODE register
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

        // Read RAMP_STAT register
        uint32_t ramp_stat = 0;
        if (!ctx.driver->read(tmc5160::Reg::RAMP_STAT, ramp_stat)) {
            logsys::printf(
                "[%s][M%u] RAMPMODE=%s (raw=0x%08lX) RAMP_STAT read failed\r\n",
                tag, static_cast<unsigned>(i), mode_str,
                mode_ok ? (unsigned long)ramp_mode : 0ul);
            continue;
        }

        // Extract RAMP_STAT fields
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

        // Read VACTUAL (actual velocity)
        uint32_t vactual = 0;
        bool vactual_ok = ctx.driver->read(tmc5160::Reg::VACTUAL, vactual);
        const char* vact_str = "?";
        char vact_buf[16];
        if (vactual_ok) {
            const float vel
                = decode_velocity(vactual, ctx.cfg.clock_frequency_hz);
            const long vactual_usteps = std::lround(vel);
            snprintf(vact_buf, sizeof(vact_buf), "%ld", vactual_usteps);
            vact_str = vact_buf;
        }

        // Read VMAX (target velocity)
        uint32_t vmax_reg = 0;
        bool vmax_ok = ctx.driver->read(tmc5160::Reg::VMAX, vmax_reg);
        const char* vmax_str = "?";
        char vmax_buf[32];
        if (vmax_ok) {
            // VMAX is unsigned 23-bit, decode differently than VACTUAL
            double vmax_vel = static_cast<double>(vmax_reg & 0x7FFFFFu)
                * static_cast<double>(ctx.cfg.clock_frequency_hz)
                / static_cast<double>(1u << 24);
            const long vmax_usteps = std::lround(vmax_vel);
            snprintf(vmax_buf, sizeof(vmax_buf), "%ld (0x%lX)", vmax_usteps,
                (unsigned long)vmax_reg);
            vmax_str = vmax_buf;
        }

        // Print everything
        logsys::printf(
            "[%s][M%u] RAMPMODE=%s (0x%08lX) RAMP_STAT=0x%08lX "
            "stopL=%lu stopR=%lu latchL=%lu latchR=%lu eventL=%lu eventR=%lu "
            "eventSG=%lu "
            "posEvt=%lu vel=%lu pos=%lu vzero=%lu vact=%s vmax=%s\r\n",
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

} // namespace motor_manager
