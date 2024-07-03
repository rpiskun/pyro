cmake_minimum_required(VERSION 3.5)
project(startup ASM)

set(LOCAL_HDRS "")
set(LOCAL_SRCS "")

set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -x assembler-with-cpp")

set(LOCAL_SRCS 
    "startup_stm32l072rbtx.s")

add_library(startup ${LOCAL_SRCS})

