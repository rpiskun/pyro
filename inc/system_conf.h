#ifndef SYSTEM_CONF_H
#define SYSTEM_CONF_H

#include "stm32l0xx_hal.h"

int clock_init_msi(void);
int clock_init_hsi(void);
int clock_init_max(void);
int gpio_init(void);
int power_init(void);

#endif /* SYSTEM_CONF_H */
