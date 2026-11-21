cmake_minimum_required(VERSION 3.18)
project(robotic_arm NONE)

# The selected μT-Kernel SMP port uses its own GNU make build. Keep the
# existing CMake entry point for the Pico VS Code workflow, but delegate the
# firmware build to that port instead of trying to link a Pico SDK-only app.
set(MTK_KERNEL_DIR "${CMAKE_CURRENT_LIST_DIR}/../third_party/mtk3smp-rp2040")
set(MTK_BUILD_DIR "${MTK_KERNEL_DIR}/build_make")

if(WIN32 OR CMAKE_HOST_WIN32)
    # The kernel Makefiles use POSIX recipes such as `mkdir -p`. Prefer
    # MSYS2's make, whose shell understands those recipes, over GnuWin32 make
    # (which can select cmd.exe and fail with "A subdirectory ... -p").
    if(EXISTS "C:/msys64/usr/bin/make.exe")
        set(MTK_MAKE_EXECUTABLE "C:/msys64/usr/bin/make.exe" CACHE FILEPATH
            "GNU make with an MSYS shell" FORCE)
    elseif(DEFINED ENV{MSYS2_ROOT} AND
           EXISTS "$ENV{MSYS2_ROOT}/usr/bin/make.exe")
        set(MTK_MAKE_EXECUTABLE "$ENV{MSYS2_ROOT}/usr/bin/make.exe" CACHE FILEPATH
            "GNU make with an MSYS shell" FORCE)
    else()
        find_program(MTK_MAKE_EXECUTABLE NAMES make mingw32-make REQUIRED)
    endif()
    set(DEFAULT_PICO_SDK_PATH "$ENV{USERPROFILE}/.pico-sdk/sdk/2.3.1")
else()
    find_program(MTK_MAKE_EXECUTABLE NAMES make gmake REQUIRED)
    set(DEFAULT_PICO_SDK_PATH "$ENV{HOME}/pico/pico-sdk")
endif()
set(PICO_SDK_PATH "${DEFAULT_PICO_SDK_PATH}" CACHE PATH
    "Pico SDK path used by μT-Kernel's USB CDC console")
file(TO_CMAKE_PATH "${PICO_SDK_PATH}" PICO_SDK_PATH_MAKE)

if(NOT EXISTS "${MTK_BUILD_DIR}/makefile")
    message(FATAL_ERROR "μT-Kernel checkout is missing: ${MTK_KERNEL_DIR}")
endif()

add_custom_target(robotic_arm ALL
    COMMAND "${MTK_MAKE_EXECUTABLE}" SMP=1 CONSOLE=usb_cdc
            WIFI=none WIFI_JOIN=0 WIFI_NETIF=0 WIFI_DHCP=0 WIFI_LOG=0
            "PICO_SDK_PATH=${PICO_SDK_PATH_MAKE}"
    WORKING_DIRECTORY "${MTK_BUILD_DIR}"
    USES_TERMINAL
    COMMENT "Building robot-arm firmware with μT-Kernel 3 SMP")

add_custom_target(robotic_arm_clean
    COMMAND "${MTK_MAKE_EXECUTABLE}" clean SMP=1 CONSOLE=usb_cdc
            "PICO_SDK_PATH=${PICO_SDK_PATH_MAKE}"
    WORKING_DIRECTORY "${MTK_BUILD_DIR}"
    USES_TERMINAL
    COMMENT "Cleaning μT-Kernel robot-arm build")

# Opt-in only; keeps the normal mission image under its original filename.
find_program(CLAW_TEST_PICOTOOL NAMES picotool
    HINTS "$ENV{USERPROFILE}/.pico-sdk/picotool/2.3.1/picotool")
set(CLAW_TEST_MAKE_ARGS)
if(CLAW_TEST_PICOTOOL)
    # Use the installed Pico tool when available; otherwise keep the BSP converter.
    list(APPEND CLAW_TEST_MAKE_ARGS "E2U=")
endif()

add_custom_target(claw_serial_test
    COMMAND "${MTK_MAKE_EXECUTABLE}" SMP=1 CONSOLE=usb_cdc CLAW_TEST=1 WIFI=none
            "PICO_SDK_PATH=${PICO_SDK_PATH_MAKE}" ${CLAW_TEST_MAKE_ARGS}
    WORKING_DIRECTORY "${MTK_BUILD_DIR}"
    USES_TERMINAL
    COMMENT "Building GP13 claw-only USB serial firmware"
    VERBATIM)
if(CLAW_TEST_PICOTOOL)
    add_custom_command(TARGET claw_serial_test POST_BUILD
        COMMAND "${CLAW_TEST_PICOTOOL}" uf2 convert
                mtk3pico_smp1_usb_cdc_claw_test.elf mtk3pico_smp1_usb_cdc_claw_test.uf2
                --family rp2040 --platform rp2040
        WORKING_DIRECTORY "${MTK_BUILD_DIR}"
        VERBATIM)
endif()
