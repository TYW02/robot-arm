# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# We are using a Raspberry Pi Pico W (RP2040)
set(PICO_BOARD pico_w CACHE STRING "Board type" FORCE)

# Import Pico SDK before project()
include(pico_sdk_import.cmake)

project(robotic_arm C CXX ASM)

# Initialise the Pico SDK
pico_sdk_init()

# Application
add_executable(robotic_arm
    main.c
)

# Libraries
target_link_libraries(robotic_arm
    pico_stdlib
    hardware_uart
    hardware_irq
    pico_util
)

# Use USB serial for printf output
pico_enable_stdio_usb(robotic_arm 1)
pico_enable_stdio_uart(robotic_arm 0) # Important: Disable stdio UART to avoid conflict

# Generate UF2/BIN/HEX files
pico_add_extra_outputs(robotic_arm)