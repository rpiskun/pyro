#ifndef PYRO_FSM_H
#define PYRO_FSM_H

#include "stm32l0xx_hal.h"
#include "pyd1588.h"

void Pyro_FSM(void);
int Pyro_UpdateConf(union Pyd1588Config pyro_conf);

#endif /* PYRO_FSM_H */
