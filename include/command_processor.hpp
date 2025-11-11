#pragma once

#include "motion_types.hpp"
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace command
{

/**
 * @brief Enumeration of all recognized command types
 *
 * These represent the different commands that can be parsed from user input
 * over the serial console.
 */
enum class Type {
    None, ///< No valid command parsed (default/error state)
    Help, ///< Display help information
    Status, ///< Query and display current motor status
    Stop, ///< Stop all motors immediately
    SetAmplitude, ///< Set motion amplitude (for future motion profiles)
    SetMicrosteps, ///< Configure motor microstepping resolution
    Run, ///< Run motor at specified velocity
    Speed, ///< Set motor speed (same as Run)
    ConstantVelocity ///< Run at constant velocity for a specific duration
};

/**
 * @brief Parsed command structure containing all command parameters
 *
 * After parsing a command string, this structure holds the command type
 * and any associated parameters (speed, direction, duration, etc.)
 */
struct Command {
    Type type = Type::None; ///< Type of command parsed
    float duration = 0.f; ///< Duration in seconds (for timed moves)
    float peak = 0.f; ///< Peak velocity in microsteps/second
    float amplitude = 0.f; ///< Motion amplitude (reserved for profiles)
    uint32_t microsteps = 0; ///< Microstepping resolution (1, 2, 4...256)
    motion::Direction direction
        = motion::Direction::Forward; ///< Movement direction (forward/reverse)
};

/**
 * @brief Parse command buffer into an actionable structure.
 * @param buffer Null-terminated buffer (can be modified in-place).
 * @return Parsed command; Command::type==None when unrecognised or invalid.
 */
inline Command parse(char* buffer)
{
    // ========================================================================
    // HELPER FUNCTION #1: Case-insensitive string comparison
    // ========================================================================
    // This lambda (anonymous function) compares two strings, ignoring
    // uppercase/lowercase Example: "HELP" matches "help" or "Help"
    auto equals_ignore_case = [](const char* a, const char* b) -> bool {
        if (!a || !b) // Safety check: if either pointer is null
            return false; //   return false (strings don't match)

        while (*a && *b) { // Loop while both strings have characters left
                           //   *a means "character pointed to by a"
                           //   The loop stops when either string ends ('\0')

            if (std::tolower(static_cast<unsigned char>(
                    *a)) // Convert current char of 'a' to lowercase
                != std::tolower(static_cast<unsigned char>(
                    *b))) // Compare with lowercase of 'b'
                return false; //   If they don't match, strings are different

            ++a; // Move to next character in string 'a'
            ++b; // Move to next character in string 'b'
        }
        return *a == '\0'
            && *b == '\0'; // Both must end at same time to be equal
                           //   If one is longer, they don't match
    };

    // ========================================================================
    // HELPER FUNCTION #2: Parse a floating-point number from a string token
    // ========================================================================
    // Converts a string like "60000" or "123.45" into a float number
    // Example: parse_float_token("60000", speed) -> speed = 60000.0
    auto parse_float_token = [](const char* token, float& out) -> bool {
        if (!token) // Safety check: if token is null pointer
            return false; //   return false (parsing failed)

        char* end = nullptr; // Pointer to track where parsing stopped
                             //   strtof will set this to the first non-number
                             //   character

        float value = std::strtof(
            token, &end); // Convert string to float
                          //   "123.45" becomes 123.45
                          //   Stops at first non-numeric character

        if (end == token) // If end hasn't moved, no digits were converted
            return false; //   This means token wasn't a valid number

        out = value; // Store the parsed value in the output parameter
        return true; // Success! Number was parsed
    };

    // ========================================================================
    // HELPER FUNCTION #3: Parse an unsigned integer from a string token
    // ========================================================================
    // Converts a string like "256" into an unsigned 32-bit integer
    // Example: parse_u32_token("256", steps) -> steps = 256
    auto parse_u32_token = [](const char* token, uint32_t& out) -> bool {
        if (!token) // Safety check: if token is null pointer
            return false; //   return false (parsing failed)

        char* end = nullptr; // Pointer to track where parsing stopped

        unsigned long value = std::strtoul(
            token, &end, 10); // Convert string to unsigned long
                              //   Third parameter (10) means base-10 (decimal)
                              //   "256" becomes 256

        if (end == token) // If end hasn't moved, no digits were converted
            return false; //   This means token wasn't a valid number

        out = static_cast<uint32_t>(
            value); // Convert unsigned long to uint32_t and store
        return true; // Success! Number was parsed
    };

    // ========================================================================
    // HELPER FUNCTION #4: Parse direction from a string token
    // ========================================================================
    // Converts words like "fwd", "forward", "rev", "reverse" into a Direction
    // enum Example: parse_direction_token("fwd", dir) -> dir = Forward
    auto parse_direction_token
        = [&](const char* token, motion::Direction& out) -> bool {
        if (!token) // Safety check: if token is null pointer
            return false; //   return false (parsing failed)

        // Check if token matches any "reverse" variation (case-insensitive)
        if (equals_ignore_case(token, "rev") // Matches "rev", "REV", "Rev"
            || equals_ignore_case(
                token, "reverse") // Matches "reverse", "REVERSE"
            || equals_ignore_case(token, "r")) { // Matches "r", "R"
            out = motion::Direction::Reverse; // Set output to Reverse direction
            return true; // Success! Direction parsed
        }

        // Check if token matches any "forward" variation (case-insensitive)
        if (equals_ignore_case(token, "fwd") // Matches "fwd", "FWD", "Fwd"
            || equals_ignore_case(
                token, "forward") // Matches "forward", "FORWARD"
            || equals_ignore_case(token, "f")) { // Matches "f", "F"
            out = motion::Direction::Forward; // Set output to Forward direction
            return true; // Success! Direction parsed
        }

        return false; // Token didn't match any direction keyword
    };

    // ========================================================================
    // MAIN PARSING LOGIC STARTS HERE
    // ========================================================================

    Command
        cmd; // Create a new Command structure (all fields default to 0/None)
             //   This will be returned at the end

    if (!buffer) // Safety check: if buffer is null pointer
        return cmd; //   return empty command (type = None)

    // ========================================================================
    // STEP 1: Extract the first word (command keyword) from the buffer
    // ========================================================================
    char* token = std::strtok(
        buffer, " "); // Split buffer by spaces, get first word
                      //   Example: "run 60000 fwd" -> token = "run"
                      //   WARNING: strtok modifies buffer in-place!
                      //   It replaces the space with '\0'

    if (!token) // If no token was found (empty string)
        return cmd; //   return empty command (type = None)

    // ========================================================================
    // STEP 2: Match the command keyword and parse its parameters
    // ========================================================================
    // Each 'if' block checks if the first word matches a command name
    // If it matches, parse the remaining parameters and return

    // -----------------------------------------------
    // COMMAND: "help"
    // -----------------------------------------------
    if (equals_ignore_case(token, "help")) { // Does first word match "help"?
        cmd.type = Type::Help; // Set command type to Help
        return cmd; // Return immediately (no parameters needed)
    }

    // -----------------------------------------------
    // COMMAND: "status"
    // -----------------------------------------------
    if (equals_ignore_case(
            token, "status")) { // Does first word match "status"?
        cmd.type = Type::Status; // Set command type to Status
        return cmd; // Return immediately (no parameters needed)
    }

    // -----------------------------------------------
    // COMMAND: "stop"
    // -----------------------------------------------
    if (equals_ignore_case(token, "stop")) { // Does first word match "stop"?
        cmd.type = Type::Stop; // Set command type to Stop
        return cmd; // Return immediately (no parameters needed)
    }

    // -----------------------------------------------
    // COMMAND: "amplitude <value>"
    // Example: "amplitude 100.5"
    // -----------------------------------------------
    if (equals_ignore_case(
            token, "amplitude")) { // Does first word match "amplitude"?
        float amp = 0.f; // Variable to store the amplitude value

        if (!parse_float_token(std::strtok(nullptr, " "),
                amp)) // Get next token and parse as float
                      //   strtok(nullptr, " ") continues from where it left off
                      //   Gets the second word from original buffer
            return cmd; // If parsing failed, return empty command

        cmd.type = Type::SetAmplitude; // Set command type
        cmd.amplitude = amp; // Store the parsed amplitude value
        return cmd; // Return the completed command
    }

    // -----------------------------------------------
    // COMMAND: "microsteps <value>"
    // Example: "microsteps 256"
    // -----------------------------------------------
    if (equals_ignore_case(
            token, "microsteps")) { // Does first word match "microsteps"?
        uint32_t steps = 0; // Variable to store microstepping value

        if (!parse_u32_token(std::strtok(nullptr, " "),
                steps)) // Get next token, parse as integer
            return cmd; // If parsing failed, return empty command

        cmd.type = Type::SetMicrosteps; // Set command type
        cmd.microsteps = steps; // Store the parsed microsteps value
        return cmd; // Return the completed command
    }

    // -----------------------------------------------
    // COMMAND: "run <speed> [direction]"
    // Example: "run 60000 fwd" or "run 60000"
    // -----------------------------------------------
    if (equals_ignore_case(token, "run")) { // Does first word match "run"?
        float speed = 0.f; // Variable to store speed value

        if (!parse_float_token(std::strtok(nullptr, " "),
                speed)) // Get second word, parse as float
            return cmd; // If parsing failed, return empty command

        motion::Direction dir
            = motion::Direction::Forward; // Default direction is Forward

        char* next = std::strtok(
            nullptr, " "); // Try to get third word (direction)
                           //   This might be nullptr if no third word exists

        if (next) { // If there IS a third word...
            motion::Direction
                tmp; //   Temporary variable to hold parsed direction
            if (parse_direction_token(
                    next, tmp)) // Try to parse it as a direction
                dir = tmp; //   If successful, use the parsed direction
                           //   If parsing fails, keep default (Forward)
        }

        cmd.type = Type::Run; // Set command type to Run
        cmd.peak = speed; // Store the speed value
        cmd.direction = dir; // Store the direction (Forward or Reverse)
        return cmd; // Return the completed command
    }

    // -----------------------------------------------
    // COMMAND: "speed <speed> [direction]"
    // Example: "speed 60000 fwd" or "speed 60000"
    // Note: This command does exactly the same as "run"
    // -----------------------------------------------
    if (equals_ignore_case(token, "speed")) { // Does first word match "speed"?
        float speed = 0.f; // Variable to store speed value

        if (!parse_float_token(std::strtok(nullptr, " "),
                speed)) // Get second word, parse as float
            return cmd; // If parsing failed, return empty command

        motion::Direction dir
            = motion::Direction::Forward; // Default direction is Forward

        char* next
            = std::strtok(nullptr, " "); // Try to get third word (direction)

        if (next) { // If there IS a third word...
            motion::Direction
                tmp; //   Temporary variable to hold parsed direction
            if (parse_direction_token(
                    next, tmp)) // Try to parse it as a direction
                dir = tmp; //   If successful, use the parsed direction
        }

        cmd.type = Type::Speed; // Set command type to Speed
        cmd.peak = speed; // Store the speed value
        cmd.direction = dir; // Store the direction
        return cmd; // Return the completed command
    }

    // -----------------------------------------------
    // COMMAND: "const <duration> <speed> [direction]"
    // Example: "const 10 60000 fwd" means run at 60000 µsteps/s forward for 10
    // seconds
    // -----------------------------------------------
    if (equals_ignore_case(token, "const")) { // Does first word match "const"?
        float duration = 0.f; // Variable to store duration in seconds
        float speed = 0.f; // Variable to store speed value

        // Parse TWO required parameters: duration and speed
        if (!parse_float_token(
                std::strtok(nullptr, " "), duration) // Get 2nd word (duration)
            || !parse_float_token(std::strtok(nullptr, " "),
                speed)) // Get 3rd word (speed)
                        //   The || means "or" - if EITHER parsing fails...
            return cmd; //   return empty command

        motion::Direction dir
            = motion::Direction::Forward; // Default direction is Forward

        char* next
            = std::strtok(nullptr, " "); // Try to get fourth word (direction)

        if (next) { // If there IS a fourth word...
            motion::Direction
                tmp; //   Temporary variable to hold parsed direction
            if (parse_direction_token(
                    next, tmp)) // Try to parse it as a direction
                dir = tmp; //   If successful, use the parsed direction
        }

        cmd.type
            = Type::ConstantVelocity; // Set command type to ConstantVelocity
        cmd.duration = duration; // Store the duration value (in seconds)
        cmd.peak = speed; // Store the speed value (microsteps/second)
        cmd.direction = dir; // Store the direction
        return cmd; // Return the completed command
    }

    // ========================================================================
    // NO MATCH FOUND
    // ========================================================================
    // If we reach here, the command didn't match any recognized keyword
    return cmd; // Return empty command (type = None)
}

} // namespace command
