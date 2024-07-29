#ifndef SYSTEM_CONF_H
#define SYSTEM_CONF_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"

int Clock_InitMsi(void);
int Clock_InitHsi(void);
int Clock_InitHsiMax(void);
int Gpio_Init(void);
int Power_Init(void);

#endif /* SYSTEM_CONF_H */
