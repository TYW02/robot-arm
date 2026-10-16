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

project(vision_test C CXX ASM)

# Initialise the Pico SDK
pico_sdk_init()

# Application
add_executable(vision_test
    vision_test.c
)

# Libraries: Link against pico_stdlib and the UART hardware library
target_link_libraries(vision_test
    pico_stdlib
    hardware_uart
)

# Use USB serial for printf output
pico_enable_stdio_usb(vision_test 1)
pico_enable_stdio_uart(vision_test 0) # Important: Disable stdio UART to avoid conflict

# Generate UF2/BIN/HEX files
pico_add_extra_outputs(vision_test)