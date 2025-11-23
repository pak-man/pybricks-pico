cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)
set(PICO_PLATFORM rp2040)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(pybricks_picow C CXX ASM)
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
    ${PBIO_TOP}/platform/picow
    ${PBTOP}/lib
    ${PBIO_TOP}/drv/pid_dma
)

# Definitions
add_compile_definitions(
    PICO_ON_DEVICE=1
    CYW43_ENABLE_BLUETOOTH=1
)

# Source files
add_executable(pybricks_picow
    main.c
    btstack_port.c
    motor_test.c
    ${PBIO_TOP}/platform/picow/platform.c
    ${PBIO_TOP}/drv/clock/clock_rp2040.c
    ${PBIO_TOP}/drv/gpio/gpio_rp2040.c
    ${PBIO_TOP}/drv/counter/counter_rp2040_pio.c
    ${PBIO_TOP}/drv/pwm/pwm_rp2040_pio.c
    # DMA PID motor control
    ${PBIO_TOP}/drv/pid_dma/pid_dma_rp2040.c
)

# Generate PIO headers (AFTER add_executable)
pico_generate_pio_header(pybricks_picow 
    ${CMAKE_CURRENT_LIST_DIR}/pio/quadrature_encoder.pio
)

# Generate PIO header for DMA PID
pico_generate_pio_header(pybricks_picow 
    ${PBIO_TOP}/drv/pid_dma/pid_dma.pio
)

# Link libraries
target_link_libraries(pybricks_picow
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
    hardware_clocks
)

pico_enable_stdio_usb(pybricks_picow 1)
pico_enable_stdio_uart(pybricks_picow 0)
pico_add_extra_outputs(pybricks_picow)
