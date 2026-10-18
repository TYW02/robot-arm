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


# ============================================================
# Board configuration
# ============================================================

set(PICO_BOARD pico_w CACHE STRING "Board type" FORCE)


# ============================================================
# Pico SDK
# ============================================================

include(pico_sdk_import.cmake)

project(robot_arm C CXX ASM)

pico_sdk_init()


# ============================================================
# MAIN ROBOT PROGRAM
# ============================================================

add_executable(robot_main
    main.c
)

target_link_libraries(robot_main
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(robot_main 1)
pico_enable_stdio_uart(robot_main 0)

pico_add_extra_outputs(robot_main)


# ============================================================
# MANUAL SERVO CALIBRATION PROGRAM
# ============================================================

add_executable(calibration
    calibration.c
)

target_link_libraries(calibration
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(calibration 1)
pico_enable_stdio_uart(calibration 0)

pico_add_extra_outputs(calibration)


# ============================================================
# STAGED / GROUPED MOTION TEST
# ============================================================

add_executable(motion
    motion.c
)

target_link_libraries(motion
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(motion 1)
pico_enable_stdio_uart(motion 0)

pico_add_extra_outputs(motion)