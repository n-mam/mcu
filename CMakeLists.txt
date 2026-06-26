cmake_minimum_required(VERSION 3.13)

SET(CMAKE_VERBOSE_MAKEFILE ON)

if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "platform not set: use -DPLATFORM=PICO or STM32")
endif()

if(PLATFORM STREQUAL "PICO")
    include(D:/pico-sdk/pico_sdk_init.cmake)
endif()

project(mcu C CXX ASM)

if(PLATFORM STREQUAL "PICO")
    include(cmake/pico.cmake)
elseif(PLATFORM STREQUAL "STM32")
    include(cmake/stm32.cmake)
else()
    message(FATAL_ERROR "Invalid PLATFORM")
endif()

include(cmake/proto.cmake)

add_executable(mcu)

target_compile_features(mcu PUBLIC cxx_std_20)

include(cmake/target.cmake)