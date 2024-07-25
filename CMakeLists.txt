cmake_minimum_required(VERSION 3.9)

project(pyro.elf)

set(LOCAL_HDRS "")
set(LOCAL_SRCS "")

add_definitions(-DMSI_VALUE=2097000u)
add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DSTM32L072xx)
add_definitions(-DDEBUG)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32L072RBTX_FLASH.ld")

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus --specs=nano.specs -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -mfloat-abi=soft -mthumb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -mfloat-abi=soft -mthumb -Wno-packed-bitfield-compat")

set(LOCAL_HDRS 
    "${CMAKE_CURRENT_SOURCE_DIR}/inc/stm32l0xx_hal_conf.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/inc/stm32l0xx_it.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/inc/rtc.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/inc/system_init.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/inc/main.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/inc/pyro_fsm.h"
)

set(LOCAL_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/stm32l0xx_it.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/system_stm32l0xx.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/syscalls.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sysmem.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/rtc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/system_init.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/pyro_fsm.c"
)

add_executable(${PROJECT_NAME} ${LOCAL_SRCS} ${LOCAL_HDRS})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/CMSIS/core/inc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/CMSIS/dev/inc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/hal/inc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/pyd1588/inc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} --specs=nano.specs -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections -Wl,--start-group -Wl,-lc -Wl,-lm -Wl,--end-group")

add_subdirectory(drivers/pyd1588/src)
add_subdirectory(drivers/hal/src)
add_subdirectory(startup)

target_link_libraries(${PROJECT_NAME} startup pyd1588 hal)

