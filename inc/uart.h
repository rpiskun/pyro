#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"

extern UART_HandleTypeDef huart2;

int Uart_Init(void);
int Uart_Send(const uint8_t *pdata, uint16_t size);
bool Uart_IsTxReady(void);

#endif /* UART_H */

