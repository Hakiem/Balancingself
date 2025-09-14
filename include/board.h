#ifndef BOARD_H
#define BOARD_H

#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
void MX_I2C1_Init(void);
void MX_USART2_UART_Init(void);
void MX_GPIO_Init(void);
void BlinkyLED(void);
void MX_TIM2_Init(void);

#endif /* BOARD_H */