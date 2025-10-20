#pragma once

#ifdef UNIT_TEST

#include <cstdint>

struct UART_HandleTypeDef {
};

struct SPI_HandleTypeDef {
};

struct TIM_HandleTypeDef {
};

struct GPIO_TypeDef {
};

enum HAL_StatusTypeDef {
    HAL_OK = 0,
    HAL_ERROR = 1,
};

constexpr uint32_t HAL_MAX_DELAY = 0xFFFFFFFFu;

constexpr uint32_t GPIO_PIN_RESET = 0u;
constexpr uint32_t GPIO_PIN_SET = 1u;

inline void HAL_GPIO_WritePin(GPIO_TypeDef*, uint16_t, uint32_t) { }

inline HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef*, uint8_t*, uint16_t, uint32_t)
{
    return HAL_OK;
}

inline HAL_StatusTypeDef HAL_SPI_Transmit(
    SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t)
{
    return HAL_OK;
}

inline HAL_StatusTypeDef HAL_SPI_Receive(
    SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t)
{
    return HAL_OK;
}

inline HAL_StatusTypeDef HAL_SPI_TransmitReceive(
    SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t, uint32_t)
{
    return HAL_OK;
}

inline HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef*) { return HAL_OK; }

inline void HAL_Delay(uint32_t) { }

inline void __NOP() { }

#define __HAL_TIM_SET_COUNTER(timer, value) (void)(timer), (void)(value)
#define __HAL_TIM_GET_COUNTER(timer) (0u)

#endif // UNIT_TEST
