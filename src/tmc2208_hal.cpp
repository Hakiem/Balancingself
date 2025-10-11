#include "tmc2208_hal.hpp"

namespace tmc2208_hal
{
namespace
{

LogFunc g_logger = nullptr;

void hal_uart_transmit(UART_HandleTypeDef* uart, const uint8_t* data, size_t len)
{
    if (g_logger) {
        g_logger("TX", data, len);
    }
    HAL_HalfDuplex_EnableTransmitter(uart);
    HAL_UART_Transmit(uart, const_cast<uint8_t*>(data), len, HAL_MAX_DELAY);
}

size_t hal_uart_receive(
    UART_HandleTypeDef* uart, uint8_t* data, size_t len, uint32_t timeout_us)
{
    uint32_t timeout_ms = (timeout_us + 999u) / 1000u;
    if (timeout_ms == 0u) {
        timeout_ms = 1u;
    }
    HAL_HalfDuplex_EnableReceiver(uart);
    size_t received
        = (HAL_UART_Receive(uart, data, len, timeout_ms) == HAL_OK) ? len : 0u;
    if (g_logger) {
        g_logger(
            received == len ? "RX" : "RX_SHORT", data, received);
    }
    return received;
}

} // namespace

tmc2208::Driver make_driver(UART_HandleTypeDef* uart, uint8_t slave_address)
{
    tmc2208::Driver::TxFunc tx
        = [uart](const uint8_t* data, size_t len) { hal_uart_transmit(uart, data, len); };
    tmc2208::Driver::RxFunc rx
        = [uart](uint8_t* data, size_t len, uint32_t timeout_us) {
              return hal_uart_receive(uart, data, len, timeout_us);
          };
    return tmc2208::Driver(tx, rx, slave_address);
}

void set_logger(LogFunc func) { g_logger = func; }

} // namespace tmc2208_hal
