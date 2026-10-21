# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(CMAKE_HOST_WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)

set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type" FORCE)

include(pico_sdk_import.cmake)
project(robot_arm C CXX ASM)
pico_sdk_init()

# Bare-metal manual calibration utility. sleep_ms() is acceptable here because
# this target is deliberately not the final RTOS application.
add_executable(calibration calibration.c)
target_link_libraries(calibration pico_stdlib hardware_i2c)
pico_enable_stdio_usb(calibration 1)
pico_enable_stdio_uart(calibration 0)
pico_add_extra_outputs(calibration)

# The final robot firmware is built by the micro T-Kernel port's make system,
# not by the Pico SDK runtime. Keeping it as a CMake target makes it selectable
# from the Pico VS Code extension without accidentally linking two runtimes.
set(MTK3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mtk3smp-rp2040")

if(CMAKE_HOST_WIN32)
    find_program(MTK3_BASH bash HINTS "C:/msys64/usr/bin" "C:/msys64/bin")
    if(MTK3_BASH)
        file(TO_CMAKE_PATH "${USERHOME}/.pico-sdk/toolchain/${toolchainVersion}/bin" MTK3_TOOLCHAIN_BIN)
        file(TO_CMAKE_PATH "${USERHOME}/.pico-sdk/sdk/${sdkVersion}" MTK3_PICO_SDK)
        file(TO_CMAKE_PATH "${MTK3_ROOT}/build_make" MTK3_BUILD_DIR)
        add_custom_target(robot_main
            COMMAND "${MTK3_BASH}" -lc
                "export PATH=\"\$(cygpath -u '${MTK3_TOOLCHAIN_BIN}'):/ucrt64/bin:/usr/bin:\$PATH\" && cd \"\$(cygpath -u '${MTK3_BUILD_DIR}')\" && make SMP=1 CONSOLE=usb_cdc PICO_SDK_PATH=\"\$(cygpath -u '${MTK3_PICO_SDK}')\" -j8"
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "${MTK3_ROOT}/build_make/mtk3pico_smp1_usb_cdc.uf2"
                "${CMAKE_BINARY_DIR}/robot_main.uf2"
            WORKING_DIRECTORY "${MTK3_ROOT}/build_make"
            COMMENT "Building robot_main with micro T-Kernel 3 SMP"
            VERBATIM
        )
    else()
        message(WARNING "MSYS2 bash not found; run make SMP=1 CONSOLE=usb_cdc in third_party/mtk3smp-rp2040/build_make")
    endif()
else()
    find_program(MTK3_MAKE make REQUIRED)
    add_custom_target(robot_main
        COMMAND "${MTK3_MAKE}" SMP=1 CONSOLE=usb_cdc PICO_SDK_PATH=$ENV{PICO_SDK_PATH} -j8
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${MTK3_ROOT}/build_make/mtk3pico_smp1_usb_cdc.uf2"
            "${CMAKE_BINARY_DIR}/robot_main.uf2"
        WORKING_DIRECTORY "${MTK3_ROOT}/build_make"
        COMMENT "Building robot_main with micro T-Kernel 3 SMP"
        VERBATIM
    )
endif()
