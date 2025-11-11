#pragma once

/**
 * @brief Command handler module
 * 
 * This module processes parsed commands and executes them on the motors.
 * It bridges the command_processor (which parses text) and motor_manager
 * (which controls the hardware).
 */

namespace command_handler
{

/**
 * @brief Process and execute a command string
 * @param buffer Command string to parse and execute (null-terminated)
 * 
 * This function:
 * 1. Parses the command string using command_processor
 * 2. Executes the command on the appropriate motors
 * 3. Logs diagnostic information (DRV_STATUS, RAMP state)
 * 
 * Supported commands:
 *   - help: Display available commands
 *   - status: Show motor status
 *   - stop: Stop all motors
 *   - amplitude <value>: Set motor current (0-255)
 *   - microsteps <value>: Set microstepping (1, 2, 4...256)
 *   - run <speed> [dir]: Run at constant speed
 *   - speed <speed> [dir]: Same as run
 *   - const <duration> <speed> [dir]: Run for specified duration
 */
void process_command(char* buffer);

} // namespace command_handler
