#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"

struct __attribute__((packed, aligned(1))) BufItem {
    uint32_t timestamp;
    int16_t adc_inst;
    int16_t adc_avg;
};

extern UART_HandleTypeDef huart2;

int Uart_Init(void);
int Uart_Deinit(void);
int Uart_Send(const struct BufItem *const pitem);
void Uart_Sender(void);
bool Uart_IsTxReady(void);

#endif /* UART_H */

