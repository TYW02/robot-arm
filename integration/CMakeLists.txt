cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Needs env vars PICO_SDK_PATH and FREERTOS_KERNEL_PATH (or -D on the cmake line).
include(pico_sdk_import.cmake)
set(ENV{FREERTOS_KERNEL_PATH} "/Users/dam/Documents/embedded/FreeRTOS-Kernel-main")
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(robotic_arm C CXX ASM)
pico_sdk_init()

# Teammate's IMU driver, used unmodified
set(IMU_DIR ${CMAKE_CURRENT_LIST_DIR}/../components/imu)

add_executable(robotic_arm
    main.c
    ${IMU_DIR}/imu.c
)

target_include_directories(robotic_arm PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}      # FreeRTOSConfig.h
    ${IMU_DIR}                     # imu.h
)

target_compile_options(robotic_arm PRIVATE -Wall -Wextra)

target_link_libraries(robotic_arm
    pico_stdlib
    hardware_i2c
    hardware_uart
    hardware_irq
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    m
)

pico_enable_stdio_usb(robotic_arm 1)
pico_enable_stdio_uart(robotic_arm 0)    # UART0 is the vision link
pico_add_extra_outputs(robotic_arm)
