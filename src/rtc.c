#include "rtc.h"

RTC_HandleTypeDef hrtc;

int Rtc_Init(void)
{
    int retval = 0;
    /** Initialize RTC Only */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        retval = -1;
    }

    return retval;
}


/* needed for HAL */
void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{
    if (rtcHandle->Instance == RTC)
    {
        /* RTC clock enable */
        __HAL_RCC_RTC_ENABLE();
    }
}

/* needed for HAL */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{
    if (rtcHandle->Instance == RTC)
    {
        /* Peripheral clock disable */
        __HAL_RCC_RTC_DISABLE();
    }
}
