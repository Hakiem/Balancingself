#include "command_handler.hpp"
#include "command_processor.hpp"
#include "logger.hpp"
#include "motor_manager.hpp"
#include <algorithm>

namespace command_handler
{

namespace
{

    // Sanitize microstep value to nearest valid setting
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
            return 256; // Default to 256 if invalid
        }
    }

} // anonymous namespace

void process_command(char* buffer)
{
    // Log the command
    logsys::printf("\r\n[CMD] %s\r\n", buffer);

    // Parse the command string
    command::Command cmd = command::parse(buffer);

    // Execute based on command type
    switch (cmd.type) {

    // -----------------------------------------------------------------------
    // HELP: Display available commands
    // -----------------------------------------------------------------------
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

    // -----------------------------------------------------------------------
    // STATUS: Query and display motor status
    // -----------------------------------------------------------------------
    case command::Type::Status:
        motor_manager::print_status();
        break;

    // -----------------------------------------------------------------------
    // STOP: Stop all motors
    // -----------------------------------------------------------------------
    case command::Type::Stop:
        motor_manager::for_each_motor(
            "STOP", [](motor_manager::MotorContext& ctx, size_t) {
                if (!ctx.driver)
                    return false;
                ctx.timed_move = false; // Cancel any timed move
                ctx.last_velocity = 0.f; // Reset velocity tracking
                return ctx.driver->stop(
                    ctx.cfg.clock_frequency_hz); // Stop the motor
            });
        break;

    // -----------------------------------------------------------------------
    // SET AMPLITUDE: Adjust motor current (0-255 scale)
    // -----------------------------------------------------------------------
    case command::Type::SetAmplitude: {
        // Clamp amplitude to 0-255 range
        const float clamped = (cmd.amplitude < 0.f)
            ? 0.f
            : (cmd.amplitude > 255.f ? 255.f : cmd.amplitude);

        // Convert to 0-31 scale for TMC5160 (IRUN register)
        const float scale = clamped / 255.f;
        const int irun
            = static_cast<int>(scale * 31.f + 0.5f); // Round to nearest
        const int ihold = (irun / 2 < 1)
            ? 1
            : (irun / 2); // Hold current = half of run current

        motor_manager::for_each_motor(
            "CUR", [&](motor_manager::MotorContext& ctx, size_t) {
                if (!ctx.driver)
                    return false;
                ctx.cfg.irun = static_cast<uint8_t>(irun);
                ctx.cfg.ihold = static_cast<uint8_t>(ihold);
                return ctx.driver->setCurrent(
                    ctx.cfg.irun, ctx.cfg.ihold, ctx.cfg.ihold_delay);
            });
        break;
    }

    // -----------------------------------------------------------------------
    // SET MICROSTEPS: Configure microstepping resolution
    // -----------------------------------------------------------------------
    case command::Type::SetMicrosteps: {
        const uint16_t micro = sanitize_microsteps(cmd.microsteps);
        motor_manager::for_each_motor(
            "MSTEPS", [&](motor_manager::MotorContext& ctx, size_t) {
                if (!ctx.driver)
                    return false;
                ctx.cfg.microsteps = micro;
                return ctx.driver->setMicrosteps(micro, ctx.cfg);
            });
        break;
    }

    // -----------------------------------------------------------------------
    // RUN / SPEED: Set motor velocity
    // -----------------------------------------------------------------------
    case command::Type::Run:
    case command::Type::Speed: {
        const float velocity
            = (cmd.peak < 0.f) ? 0.f : cmd.peak; // Ensure non-negative
        const motion::Direction dir = cmd.direction;

        motor_manager::for_each_motor(
            "SPEED", [&](motor_manager::MotorContext& ctx, size_t) {
                if (!ctx.driver)
                    return false;
                ctx.timed_move = false; // This is NOT a timed move
                return motor_manager::apply_velocity(ctx, velocity, dir);
            });
        break;
    }

    // -----------------------------------------------------------------------
    // CONST: Run at constant velocity for specified duration
    // -----------------------------------------------------------------------
    case command::Type::ConstantVelocity: {
        const float velocity
            = (cmd.peak < 0.f) ? 0.f : cmd.peak; // Ensure non-negative
        const motion::Direction dir = cmd.direction;

        // Convert duration from seconds to milliseconds
        const uint32_t duration_ms = static_cast<uint32_t>(
            (cmd.duration > 0.f ? cmd.duration : 0.f) * 1000.f);

        motor_manager::for_each_motor(
            "CONST", [&](motor_manager::MotorContext& ctx, size_t) {
                if (!ctx.driver)
                    return false;

                // Apply the velocity
                if (!motor_manager::apply_velocity(ctx, velocity, dir))
                    return false;

                // Set up timed move if duration > 0
                if (duration_ms > 0u) {
                    ctx.timed_move = true;
                    ctx.deadline_ms
                        = HAL_GetTick() + duration_ms; // Calculate deadline
                } else {
                    ctx.timed_move = false; // No timeout, run indefinitely
                }
                return true;
            });
        break;
    }

    // -----------------------------------------------------------------------
    // NONE / UNKNOWN: Invalid or unrecognized command
    // -----------------------------------------------------------------------
    case command::Type::None:
    default:
        logsys::printf("[CMD] Unknown. Type 'help'.\r\n");
        break;
    }

    // After executing command, log diagnostic information
    motor_manager::log_drv_status("DRV"); // Driver status
    motor_manager::log_motion_state("RAMP"); // Ramp generator state
}

} // namespace command_handler
