#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include "stm32l0xx_hal.h"

int clock_init_msi(void);
int clock_init_hsi(void);
int clock_init_max(void);
int gpio_init(void);

#endif /* SYSTEM_INIT_H */
