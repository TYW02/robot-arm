cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Target board")

# Supply PICO_SDK_PATH through CMake or the environment.
if(NOT PICO_SDK_PATH)
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()

if(NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR
        "Set PICO_SDK_PATH to your installed Pico SDK directory")
endif()

include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(imu_test C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(imu_test
    main.c
)

target_link_libraries(imu_test
    pico_stdlib
    hardware_i2c
    m
)

pico_enable_stdio_usb(imu_test 1)
pico_enable_stdio_uart(imu_test 0)

pico_add_extra_outputs(imu_test)