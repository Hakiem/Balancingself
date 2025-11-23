#include "console.hpp"
#include "main.h"
#include <cstring>

namespace console
{

namespace
{

    // UART handle (set during initialization)
    UART_HandleTypeDef* uart_handle = nullptr;

    // Command buffer for storing user input
    constexpr size_t kCommandBufferSize = 128;
    char cmd_buffer[kCommandBufferSize];
    uint8_t cmd_index = 0; // Current position in buffer
    bool cmd_ready_flag = false; // True when command is ready to process

} // anonymous namespace

void initialize(void* uart_handle_ptr)
{
    uart_handle = static_cast<UART_HandleTypeDef*>(uart_handle_ptr);
    cmd_index = 0;
    cmd_ready_flag = false;
}

void handle_input()
{
    if (!uart_handle)
        return;

    // Try to receive one byte (non-blocking)
    uint8_t byte = 0;
    if (HAL_UART_Receive(uart_handle, &byte, 1, 0) != HAL_OK)
        return; // No data available or error

    // Check for newline (Enter key)
    if (byte == '\r' || byte == '\n') {
        if (cmd_index > 0) {
            // Null-terminate the command string
            cmd_buffer[cmd_index] = '\0';
            cmd_ready_flag = true; // Mark command as ready
        }
        cmd_index = 0; // Reset buffer for next command
        return;
    }

    // Check if character is printable and buffer has space
    if (byte >= 32 && byte <= 126 && cmd_index < kCommandBufferSize - 1) {
        cmd_buffer[cmd_index++] = static_cast<char>(byte); // Add to buffer

        // Echo character back to terminal
        HAL_UART_Transmit(uart_handle, &byte, 1, HAL_MAX_DELAY);
    }
}

bool command_ready() { return cmd_ready_flag; }

char* get_command_buffer() { return cmd_buffer; }

void clear_command() { cmd_ready_flag = false; }

void write(const char* msg)
{
    if (!uart_handle || !msg)
        return;

    // Transmit the entire string
    HAL_UART_Transmit(uart_handle, reinterpret_cast<const uint8_t*>(msg),
        static_cast<uint16_t>(std::strlen(msg)), HAL_MAX_DELAY);
}

void print_banner() { write("\r\n=== Self Balancing Robot Demo ===\r\n"); }

} // namespace console
