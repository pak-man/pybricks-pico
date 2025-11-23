
# ============================================================================
# Pybricks Motor Control - Unified Build for Pico W and Pico 2 W
# ============================================================================
#
# Usage:
#   mkdir build && cd build
#
#   # Build for Pico W (RP2040):
#   cmake -DTARGET_BOARD=picow ..
#   make
#
#   # Build for Pico 2 W (RP2350B):
#   cmake -DTARGET_BOARD=pico2w ..
#   make
#
#   # Build both (requires separate build dirs):
#   mkdir build_picow build_pico2w
#   cd build_picow && cmake -DTARGET_BOARD=picow .. && make
#   cd ../build_pico2w && cmake -DTARGET_BOARD=pico2w .. && make
#
# ============================================================================

# Default to Pico W if not specified
if(NOT DEFINED TARGET_BOARD)
    set(TARGET_BOARD "picow")
    message(STATUS "TARGET_BOARD not specified, defaulting to: picow")
endif()

# Configure based on target
if(TARGET_BOARD STREQUAL "pico2w")
    set(PICO_BOARD pico2_w)
    set(PICO_PLATFORM rp2350)
    set(PROJECT_NAME pybricks_pico2w)
    set(NUM_MOTORS 12)
    set(IS_PICO_2W TRUE)
else()
    set(PICO_BOARD pico_w)
    set(PICO_PLATFORM rp2040)
    set(PROJECT_NAME pybricks_picow)
    set(NUM_MOTORS 4)
    set(IS_PICO_2W FALSE)
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(${PROJECT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

# Paths
set(PBTOP ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PBIO_TOP ${PBTOP}/lib/pbio)

# Include directories
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PBIO_TOP}/drv/pid_dma
    ${PBIO_TOP}/platform/pico_common
)

# Common definitions
add_compile_definitions(
    PICO_ON_DEVICE=1
    CYW43_ENABLE_BLUETOOTH=1
    PYBRICKS_MOTOR_COUNT=${NUM_MOTORS}
    PID_DMA_MAX_MOTORS=${NUM_MOTORS}
)

# Platform-specific definitions
if(IS_PICO_2W)
    add_compile_definitions(
        PICO_2W=1
        PICO_RP2350=1
        PICO_RP2350B=1
    )
endif()

# Source files - pid_dma handles everything
add_executable(${PROJECT_NAME}
    main.c
    btstack_port.c
    motor_test.c
    ${PBIO_TOP}/drv/pid_dma/pid_dma_rp2040.c
)

# Generate PIO header for encoder (in pid_dma)
pico_generate_pio_header(${PROJECT_NAME} 
    ${PBIO_TOP}/drv/pid_dma/pid_dma.pio
)

# Link libraries (same for both platforms)
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_cyw43_arch_none
    pico_btstack_ble
    pico_btstack_cyw43
    pico_multicore
    hardware_gpio
    hardware_pio
    hardware_pwm
    hardware_timer
    hardware_dma
    hardware_watchdog
    hardware_clocks
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME})

# Build summary
message(STATUS "============================================")
message(STATUS "Building: ${PROJECT_NAME}")
message(STATUS "  Board: ${PICO_BOARD}")
message(STATUS "  Platform: ${PICO_PLATFORM}")
message(STATUS "  Motors: ${NUM_MOTORS}")
message(STATUS "  Control: Dual-core @ 1 kHz")
if(IS_PICO_2W)
    message(STATUS "  GPIO: 48 pins")
else()
    message(STATUS "  GPIO: 30 pins")
endif()
message(STATUS "============================================")
