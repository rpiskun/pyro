#include "stm32l0xx_hal.h"
#include "stm32l0xx_it.h"
#include "pyd1588.h"
#include "uart.h"
/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable Interrupt.
  */
void NMI_Handler(void)
{
    while (1)
    {
    }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    while (1)
    {
    }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
  * @brief  This function handles external lines 0 to 1 interrupt request.
  */
void EXTI0_1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PYD_DIRECT_LINK_PIN);
}

/**
  * @brief  This function handles DMA interrupt request.
  */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(huart2.hdmatx);
}

/**
  * @brief  This function handles USART1 interrupt request.
  */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

