cmake_minimum_required(VERSION 3.9)

project(pyd1588)

set(LOCAL_HDRS "")
set(LOCAL_SRCS "")

add_definitions(-DMSI_VALUE=2097000u)
add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DSTM32L072xx)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -mfloat-abi=soft -mthumb")

set(LOCAL_HDRS 
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc/pyd1588.h"
)

set(LOCAL_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/pyd1588.c"
)

add_library(${PROJECT_NAME} ${LOCAL_SRCS} ${LOCAL_HDRS})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../CMSIS/core/inc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../CMSIS/dev/inc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../hal/inc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../inc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../inc
)

