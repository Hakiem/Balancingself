#pragma once

#include <cstdint>

/**
 * @brief Console/UART interface module
 *
 * Handles serial console input/output, command buffering,
 * and echo functionality for interactive command entry.
 */

namespace console
{

/**
 * @brief Initialize the console module
 * @param uart Handle to the UART peripheral (typically UART2)
 *
 * Must be called before using other console functions.
 */
void initialize(void* uart_handle);

/**
 * @brief Handle incoming UART data (non-blocking)
 *
 * Call this frequently from the main loop. It will:
 * - Read one byte from UART (if available)
 * - Echo the character back to the terminal
 * - Add it to the command buffer
 * - Detect newline (Enter key) and mark command as ready
 *
 * Non-blocking: Returns immediately if no data is available.
 */
void handle_input();

/**
 * @brief Check if a complete command is ready to process
 * @return true if user pressed Enter and command is ready
 */
bool command_ready();

/**
 * @brief Get the command buffer
 * @return Pointer to null-terminated command string
 *
 * Only call this if command_ready() returns true.
 * After processing, call clear_command() to reset the buffer.
 */
char* get_command_buffer();

/**
 * @brief Clear the command ready flag
 *
 * Call this after processing a command to allow
 * the next command to be received.
 */
void clear_command();

/**
 * @brief Write a string to the console
 * @param msg Null-terminated string to send
 *
 * Blocks until all characters are transmitted.
 */
void write(const char* msg);

/**
 * @brief Print the welcome banner
 *
 * Displays the application name/version when the system starts.
 */
void print_banner();

} // namespace console
