#include "command_handler.hpp"
#include "command_processor.hpp"
#include "logger.hpp"
#include <algorithm>

namespace command_handler
{

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
        logsys::printf("=== Available Commands ===\r\n");
        logsys::printf("  help   - Show this help\r\n");
        logsys::printf("  status - Show system status\r\n");
        logsys::printf("\r\n[TODO] Commands to be implemented:\r\n");
        logsys::printf("  - IMU calibration (MPU9250)\r\n");
        logsys::printf("  - Motor control (DRV8256E)\r\n");
        logsys::printf("  - Encoder readings\r\n");
        logsys::printf("  - PID tuning\r\n");
        logsys::printf("  - Wireless config (NRF24L01)\r\n");
        break;

    // -----------------------------------------------------------------------
    // STATUS: Show system information
    // -----------------------------------------------------------------------
    case command::Type::Status:
        logsys::printf("[STATUS] System: OK\r\n");
        logsys::printf("[STATUS] Uptime: %lu ms\r\n", HAL_GetTick());
        logsys::printf("[STATUS] MCU: STM32F303K8\r\n");
        logsys::printf("[TODO] Add sensor readings\r\n");
        break;

    // -----------------------------------------------------------------------
    // UNKNOWN: Command not recognized
    // -----------------------------------------------------------------------
    default:
        logsys::printf(
            "[ERROR] Unknown command. Type 'help' for available commands.\r\n");
        break;
    }
}

} // namespace command_handler
