#ifndef PYRO_FSM_H
#define PYRO_FSM_H

#include <stdint.h>
#include "stm32l0xx_hal.h"
#include "pyd1588.h"

struct AdcInstantValue {
    uint32_t timestamp;
    int16_t adc_value;
};

int Pyro_Init(void);
int Pyro_UpdateConf(union Pyd1588Config pyro_conf);
bool Pyro_IsConfUpdated(void);
void Pyro_Fsm(void);
int Pyro_StartAdcRead(void);
int Pyro_StopAdcRead(void);
bool Pyro_GetAdcValue(struct AdcInstantValue *padcval);

#endif /* PYRO_FSM_H */
