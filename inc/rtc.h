#ifndef RTC_H
#define RTC_H

#include "stm32l0xx_hal.h"

int rtc_init(void);
void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle);
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle);

#endif /* RTC_H */

