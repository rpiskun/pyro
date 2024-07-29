cmake_minimum_required(VERSION 3.9)

project(hal)

set(LOCAL_HDRS "")
set(LOCAL_SRCS "")

add_definitions(-DMSI_VALUE=2097000u)
add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DSTM32L072xx)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus --specs=nano.specs -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -mfloat-abi=soft -mthumb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -mfloat-abi=soft -mthumb")

set(LOCAL_HDRS 
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_cortex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_def.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_dma.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_exti.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_flash_ex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_flash.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_flash_ramfunc.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_gpio_ex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_gpio.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_i2c_ex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_i2c.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_pwr_ex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_pwr.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_rcc_ex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_rcc.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_rtc_ex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_rtc.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_tim_ex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_tim.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_uart.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_hal_uart_ex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_ll_bus.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_ll_cortex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_ll_crs.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_ll_dma.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_ll_exti.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_ll_gpio.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_ll_pwr.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_ll_rcc.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_ll_rtc.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_ll_system.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/stm32l0xx_ll_utils.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/Legacy/stm32_hal_legacy.h"
)

set(LOCAL_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_cortex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_dma.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_exti.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_flash.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_flash_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_flash_ramfunc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_gpio.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_i2c.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_i2c_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_pwr.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_pwr_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_rcc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_rcc_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_rtc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_rtc_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_tim.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_tim_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_uart.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx_hal_uart_ex.c"
)

add_library(${PROJECT_NAME} ${LOCAL_SRCS} ${LOCAL_HDRS})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../CMSIS/core/inc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../CMSIS/dev/inc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../inc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../inc
)

