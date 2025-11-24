cmake_minimum_required(VERSION 3.13)

# ============================================================================
# Board configuration based on TARGET_BOARD variable
# ============================================================================

if(NOT DEFINED TARGET_BOARD)
    set(TARGET_BOARD "picow")
    message(WARNING "TARGET_BOARD not set, defaulting to picow")
endif()

if(TARGET_BOARD STREQUAL "picow")
    set(PICO_BOARD pico_w)
    set(PICO_PLATFORM rp2040)
    set(NUM_MOTORS 4)
    set(IS_PICO_2W 0)
    set(OUTPUT_NAME pybricks_picow)
    message(STATUS "Building for Pico W (RP2040, ${NUM_MOTORS} motors)")
elseif(TARGET_BOARD STREQUAL "pico2w")
    set(PICO_BOARD pico2_w)
    set(PICO_PLATFORM rp2350)
    set(NUM_MOTORS 12)
    set(IS_PICO_2W 1)
    set(OUTPUT_NAME pybricks_pico2w)
    message(STATUS "Building for Pico 2 W (RP2350B, ${NUM_MOTORS} motors)")
else()
    message(FATAL_ERROR "Unknown TARGET_BOARD: ${TARGET_BOARD}. Use 'picow' or 'pico2w'")
endif()

# ============================================================================
# Import Pico SDK
# ============================================================================

# PICO_SDK_PATH must be set
if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "PICO_SDK_PATH environment variable not set")
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# ============================================================================
# Project setup
# ============================================================================

project(${OUTPUT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ============================================================================
# Paths
# ============================================================================

set(PBTOP ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PBIO_TOP ${PBTOP}/lib/pbio)

# ============================================================================
# Include directories
# ============================================================================

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PBIO_TOP}/include
    ${PBIO_TOP}/platform/pico_common
    ${PBTOP}/lib
    ${PBIO_TOP}/drv/pid_dma
    ${PBIO_TOP}/drv/watchdog
    ${PBIO_TOP}/drv/battery
    ${PBIO_TOP}/drv/ioport
    ${PBIO_TOP}/drv/storage
)

# ============================================================================
# Compile definitions
# Note: NUM_MOTORS is defined in pbdrvconfig.h based on PICO_2W
# ============================================================================

# Only define PICO_2W flag for conditional compilation

if(IS_PICO_2W)
    add_compile_definitions(PICO_2W=1)
endif()

# ============================================================================
# Source files
# ============================================================================

add_executable(${OUTPUT_NAME}
    # Main application
    main.c
    # motor_test.c  # TODO: Add motor control functions
    system_test.c
    btstack_port.c
    
    # Platform
    ${PBIO_TOP}/platform/pico_common/platform.c
    
    # Drivers
    ${PBIO_TOP}/drv/pid_dma/pid_dma_rp2040.c
    ${PBIO_TOP}/drv/imu/imu_bno085.c
    ${PBIO_TOP}/drv/watchdog/watchdog_rp2040.c
    ${PBIO_TOP}/drv/battery/battery_rp2040_adc.c
    ${PBIO_TOP}/drv/ioport/ioport_rp2040.c
    ${PBIO_TOP}/drv/storage/storage_flash_rp2040.c
    ${PBIO_TOP}/drv/storage/storage_sd_rp2040.c
    ${PBIO_TOP}/drv/storage/storage.c
)

# ============================================================================
# Generate PIO headers
# ============================================================================

pico_generate_pio_header(${OUTPUT_NAME} 
    ${PBIO_TOP}/drv/pid_dma/pid_dma.pio
)

# ============================================================================
# Link libraries
# ============================================================================

target_link_libraries(${OUTPUT_NAME}
    pico_stdlib
    pico_cyw43_arch_none
    pico_btstack_ble
    pico_btstack_cyw43
    pico_multicore
    hardware_adc
    hardware_i2c
    hardware_uart
    hardware_spi
    hardware_flash
    hardware_sync
    hardware_pwm
    hardware_pio
)

# ============================================================================
# Output generation
# ============================================================================

pico_add_extra_outputs(${OUTPUT_NAME})

# Enable USB output for debugging (optional)
# pico_enable_stdio_usb(${OUTPUT_NAME} 1)
# pico_enable_stdio_uart(${OUTPUT_NAME} 0)

# ============================================================================
# Post-build info
# ============================================================================

message(STATUS "Build succeeded, configuration:")
message(STATUS "  Target: ${TARGET_BOARD}")
message(STATUS "  Board: ${PICO_BOARD}")
message(STATUS "  Platform: ${PICO_PLATFORM}")
message(STATUS "  Motors: ${NUM_MOTORS}")
message(STATUS "  Output: ${OUTPUT_NAME}.uf2")