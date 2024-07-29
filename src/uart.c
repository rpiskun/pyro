#include "stm32l0xx_hal.h"
#include "uart.h"

enum TxStatus {
    E_TX_STATUS_READY = 0,
    E_TX_STATUS_BUSY
};

UART_HandleTypeDef huart2 = { 
        .Instance = USART2,
        .Init.BaudRate = 115200,
        .Init.WordLength = UART_WORDLENGTH_8B,
        .Init.StopBits = UART_STOPBITS_1,
        .Init.Parity = UART_PARITY_NONE,
        .Init.HwFlowCtl = UART_HWCONTROL_NONE,
        .Init.Mode = UART_MODE_TX,
        .AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT
};

static volatile enum TxStatus tx_status;

int Uart_Init(void)
{
    int retval = 0;

    do {
        if (HAL_UART_DeInit(&huart2) != HAL_OK) {
            retval = -1;
            break;
        }

        if (HAL_UART_Init(&huart2) != HAL_OK) {
            retval = -1;
            break;
        }
    } while (0);

    return retval;
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    static DMA_HandleTypeDef hdma_tx;
    GPIO_InitTypeDef GPIO_InitStruct;

    /* enable peripheral clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* UART common pin configuration */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance = DMA1_Channel4;
    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode = DMA_NORMAL;
    hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tx.Init.Request = DMA_REQUEST_4;

    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmatx, hdma_tx);

    /* NVIC configuration for DMA transfer complete interrupt (USART1_TX) */
    HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

    /* NVIC for USART1, to catch the TX complete */
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    __HAL_RCC_USART2_FORCE_RESET();
    __HAL_RCC_USART2_RELEASE_RESET();

    /* De-Initialize USART1 Tx */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
    /* De-Initialize USART1 Rx */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

    /* De-Initialize the DMA channel associated to reception process */
    if(huart->hdmarx != 0)
    {
        HAL_DMA_DeInit(huart->hdmarx);
    }
    /* De-Initialize the DMA channel associated to transmission process */
    if(huart->hdmatx != 0)
    {
        HAL_DMA_DeInit(huart->hdmatx);
    }  

    HAL_NVIC_DisableIRQ(DMA1_Channel4_5_6_7_IRQn);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Set transmission flag: transfer complete */
    tx_status = E_TX_STATUS_READY;

}

int Uart_Send(const uint8_t *pdata, uint16_t size)
{
    int retval = 0;

    if (HAL_UART_Transmit_DMA(&huart2, pdata, size) != HAL_OK) {
        retval = -1;
    }

    tx_status = E_TX_STATUS_BUSY;

    return retval; 
}

bool Uart_IsTxReady(void)
{
    bool retval = false;

    if (tx_status == E_TX_STATUS_READY) {
        retval = true;
    }

    return retval;
}

