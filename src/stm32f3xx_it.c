#include "stm32f3xx_it.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

#include <stdbool.h>

extern volatile bool g_imu_int_flag;

void NMI_Handler(void)
{
    while (1) { }
}

void HardFault_Handler(void)
{
    while (1) { }
}

void MemManage_Handler(void)
{
    while (1) { }
}

void BusFault_Handler(void)
{
    while (1) { }
}

void UsageFault_Handler(void)
{
    while (1) { }
}

void SVC_Handler(void) { }

void DebugMon_Handler(void) { }

void PendSV_Handler(void) { }

void SysTick_Handler(void) { HAL_IncTick(); }

void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }

void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12); }

void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        g_imu_int_flag = true;
    }
}
