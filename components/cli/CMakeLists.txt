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

# Standalone test build for the CLI component (no FreeRTOS).
# Same SDK settings as integration/CMakeLists.txt.
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_BOARD pico_w CACHE STRING "Board type" FORCE)

# Reuse the SDK import file already in the integration folder
include(${CMAKE_CURRENT_LIST_DIR}/../../integration/pico_sdk_import.cmake)

project(cli_test C CXX ASM)

pico_sdk_init()

add_executable(cli_test
    cli_test.c
    cli.c
)

target_include_directories(cli_test PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(cli_test PRIVATE -Wall -Wextra)
target_link_libraries(cli_test pico_stdlib)

pico_enable_stdio_usb(cli_test 1)
pico_enable_stdio_uart(cli_test 0)   # UART0 is reserved for the vision link

pico_add_extra_outputs(cli_test)
