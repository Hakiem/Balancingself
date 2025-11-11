#pragma once

#include "logger.hpp"
#include "motion_types.hpp"
#include "tmc5160_motor.hpp"
#include <cstddef>
#include <cstdint>

namespace motor_manager
{

/**
 * @brief Context structure for each motor instance
 *
 * Contains the driver pointer, configuration, and state tracking
 * for timed moves and last commanded velocity/direction.
 */
struct MotorContext {
    tmc5160::Motor* driver = nullptr; ///< Pointer to TMC5160 driver instance
    tmc5160::Motor::Config
        cfg {}; ///< Driver configuration (currents, ramp params, etc.)
    bool timed_move = false; ///< True if motor is executing a timed move
    uint32_t deadline_ms = 0; ///< Timestamp when timed move should stop
    float last_velocity = 0.f; ///< Last commanded velocity in microsteps/sec
    motion::Direction last_direction
        = motion::Direction::Forward; ///< Last commanded direction
};

/**
 * @brief Initialize all motor contexts (drivers, config, SPI communication)
 *
 * This function:
 * - Creates driver instances for each motor
 * - Configures default settings (currents, microsteps, ramp parameters)
 * - Verifies SPI communication by reading IOIN register
 * - Validates TMC5160 version and power status
 */
void initialize();

/**
 * @brief Get the motor context for a specific motor index
 * @param index Motor index (0-based)
 * @return Pointer to motor context, or nullptr if index is invalid
 */
MotorContext* get_context(size_t index);

/**
 * @brief Get the total number of motors in the system
 * @return Number of motors
 */
size_t get_motor_count();

/**
 * @brief Apply a velocity command to a motor
 * @param ctx Motor context to command
 * @param velocity_usteps_s Velocity in microsteps per second
 * @param dir Direction (Forward or Reverse)
 * @return true if command succeeded, false otherwise
 *
 * This function calls the TMC5160 setVelocity() method and updates
 * the context's last_velocity and last_direction fields.
 */
bool apply_velocity(
    MotorContext& ctx, float velocity_usteps_s, motion::Direction dir);

/**
 * @brief Service all motors with active timed moves
 *
 * Checks each motor's deadline and stops it if the time has elapsed.
 * Call this regularly from the main loop (every few milliseconds).
 */
void service_timed_moves();

/**
 * @brief Execute a function on all motors
 * @tparam Fn Function type (lambda or function pointer)
 * @param tag Label to print in log output (e.g., "SPEED", "STOP")
 * @param fn Function to execute: bool fn(MotorContext& ctx, size_t index)
 *
 * Iterates through all motors and executes the provided function.
 * Logs "OK" or "FAIL" for each motor based on the return value.
 */
template <typename Fn> void for_each_motor(const char* tag, Fn&& fn);

/**
 * @brief Print detailed status for all motors
 *
 * Displays:
 * - Current velocity (VACTUAL register)
 * - Driver status (DRV_STATUS: current, stallguard, errors)
 * - IO pin states (IOIN register)
 * - Direction and timed move status
 */
void print_status();

/**
 * @brief Log DRV_STATUS register for all motors
 * @param tag Label to print in log output
 *
 * Displays all DRV_STATUS fields: current scaler, stallguard result,
 * stealth/spreadCycle mode, error flags, etc.
 */
void log_drv_status(const char* tag);

/**
 * @brief Log motion state (RAMPMODE, RAMP_STAT, VACTUAL, VMAX) for all motors
 * @param tag Label to print in log output
 *
 * Displays ramp generator state, velocity reached flags, and current
 * actual velocity vs target velocity.
 */
void log_motion_state(const char* tag);

// ============================================================================
// Template Implementation (must be in header)
// ============================================================================

/**
 * @brief Execute a function on all motors (template implementation)
 *
 * This iterates through all motor contexts and executes the provided function,
 * logging the result for each motor.
 */
template <typename Fn> void for_each_motor(const char* tag, Fn&& fn)
{
    for (size_t i = 0; i < get_motor_count(); ++i) {
        auto* ctx_ptr = get_context(i);
        if (!ctx_ptr || !ctx_ptr->driver)
            continue;

        bool ok = fn(*ctx_ptr, i);
        logsys::printf("[%s][M%u] %s\r\n", tag, static_cast<unsigned>(i),
            ok ? "OK" : "FAIL");
    }
}

} // namespace motor_manager
