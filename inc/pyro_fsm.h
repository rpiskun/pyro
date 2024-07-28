#ifndef PYRO_FSM_H
#define PYRO_FSM_H

#include "stm32l0xx_hal.h"
#include "pyd1588.h"

int Pyro_Init(void);
int Pyro_UpdateConf(union Pyd1588Config pyro_conf);
bool Pyro_IsConfUpdated(void);
void Pyro_Fsm(void);
int Pyro_StartAdcRead(void);
int Pyro_StopAdcRead(void);
bool Pyro_GetAdcValue(int16_t *adc_val);

#endif /* PYRO_FSM_H */
