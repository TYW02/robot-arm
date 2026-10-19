
cmake_minimum_required(VERSION 3.13)

# Set the board to Raspberry Pi Pico W
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Import the installed Raspberry Pi Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Project name
project(imu_test C CXX ASM)

# Initialise Pico SDK
pico_sdk_init()

# Create executable
add_executable(imu_test
    imu_test.c
    imu.c
    imu_verify.c
)

# Link the required libraries
target_link_libraries(imu_test pico_stdlib hardware_i2c m)

# Enable USB serial communication
pico_enable_stdio_usb(imu_test 1)
pico_enable_stdio_uart(imu_test 0)

# Generate UF2 file for Raspberry Pi Pico
pico_add_extra_outputs(imu_test)
