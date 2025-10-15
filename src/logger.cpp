#include "logger.hpp"
#include <cstdio>
#include <cstring>

namespace
{
UART_HandleTypeDef* g_huart = nullptr;
}

namespace logsys
{
void init(UART_HandleTypeDef* uart) { g_huart = uart; }

void printf(const char* format, ...)
{
    if (!g_huart) {
        return;
    }

    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0) {
        size_t to_send = (len < static_cast<int>(sizeof(buffer)))
            ? len
            : sizeof(buffer) - 1;
        HAL_UART_Transmit(g_huart, reinterpret_cast<uint8_t*>(buffer),
            static_cast<uint16_t>(to_send), HAL_MAX_DELAY);
    }
}

void dump(const char* tag, const uint8_t* data, size_t len)
{
    if (!g_huart)
        return;
    char line[64];
    int n = snprintf(
        line, sizeof(line), "[%s] %u bytes: ", tag, (unsigned int)len);
    HAL_UART_Transmit(
        g_huart, reinterpret_cast<uint8_t*>(line), n, HAL_MAX_DELAY);

    for (size_t i = 0; i < len; ++i) {
        char hex[4];
        snprintf(hex, sizeof(hex), "%02X ", data[i]);
        HAL_UART_Transmit(
            g_huart, reinterpret_cast<uint8_t*>(hex), 3, HAL_MAX_DELAY);
    }
    const char newline[] = "\r\n";
    HAL_UART_Transmit(g_huart, (uint8_t*)newline, 2, HAL_MAX_DELAY);
}

} // namespace logsys