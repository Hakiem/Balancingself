#pragma once

#ifdef UNIT_TEST

#include <cstdint>

struct UART_HandleTypeDef {
};

struct SPI_HandleTypeDef {
};

struct TIM_HandleTypeDef {
    uint32_t Channel = 0;
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
inline HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }

inline void HAL_Delay(uint32_t) { }

inline void __NOP() { }

inline void __HAL_TIM_SET_COUNTER(TIM_HandleTypeDef*, uint32_t) { }
inline uint32_t __HAL_TIM_GET_COUNTER(TIM_HandleTypeDef*) { return 0u; }
inline void __HAL_TIM_SET_COMPARE(TIM_HandleTypeDef*, uint32_t, uint32_t) { }
inline uint32_t __HAL_TIM_GET_COMPARE(TIM_HandleTypeDef*, uint32_t) { return 0u; }
inline uint32_t __HAL_TIM_GET_ACTIVE_CHANNEL(TIM_HandleTypeDef* htim)
{
    return htim ? htim->Channel : 0u;
}

#define TIM_CHANNEL_1 0x00000001u
#define TIM_CHANNEL_2 0x00000002u
#define HAL_TIM_ACTIVE_CHANNEL_1 TIM_CHANNEL_1
#define HAL_TIM_ACTIVE_CHANNEL_2 TIM_CHANNEL_2

#endif // UNIT_TEST
