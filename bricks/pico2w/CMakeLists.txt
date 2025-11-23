cmake_minimum_required(VERSION 3.13)

# Set board to Pico 2 W
set(PICO_BOARD pico2_w)
set(PICO_PLATFORM rp2350)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pybricks_pico2w C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

# Paths
set(PBTOP ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PBIO_TOP ${PBTOP}/lib/pbio)

# Include directories
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PBIO_TOP}/include
    ${PBIO_TOP}/platform/pico2w
    ${PBTOP}/lib
)

# Definitions for RP2350B (48 GPIO variant)
add_compile_definitions(
    PICO_ON_DEVICE=1
    PICO_RP2350=1
    PICO_RP2350B=1
    CYW43_ENABLE_BLUETOOTH=1
    PYBRICKS_MOTOR_COUNT=12
)

# Source files
add_executable(pybricks_pico2w
    main.c
    btstack_port.c
    motor_test.c
    ${PBIO_TOP}/platform/pico2w/platform.c
    ${PBIO_TOP}/drv/clock/clock_rp2350.c
    ${PBIO_TOP}/drv/gpio/gpio_rp2350.c
    ${PBIO_TOP}/drv/counter/counter_rp2350_pio.c
    ${PBIO_TOP}/drv/pwm/pwm_rp2350_pio.c
)

# Generate PIO header
pico_generate_pio_header(pybricks_pico2w 
    ${CMAKE_CURRENT_LIST_DIR}/pio/quadrature_encoder.pio
)

# Link libraries (RP2350 has same library names)
target_link_libraries(pybricks_pico2w
    pico_stdlib
    pico_cyw43_arch_none
    pico_btstack_ble
    pico_btstack_cyw43
    hardware_gpio
    hardware_pio
    hardware_pwm
    hardware_timer
    hardware_dma
    hardware_watchdog
    pico_multicore  # For dual-core profile execution
)

pico_enable_stdio_usb(pybricks_pico2w 1)
pico_enable_stdio_uart(pybricks_pico2w 0)

pico_add_extra_outputs(pybricks_pico2w)

# Print build info
message(STATUS "Building for Pico 2 W (RP2350B)")
message(STATUS "  Motors: 12")
message(STATUS "  Encoders: 12 (PIO-based)")
message(STATUS "  PWM Slices: 12")
message(STATUS "  PIO State Machines: 12")
message(STATUS "  GPIO Pins: 48")
