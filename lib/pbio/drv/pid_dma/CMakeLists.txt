# SPDX-License-Identifier: MIT
# DMA-accelerated PID motor control for RP2040/RP2350

# Check if we're building for RP2040/RP2350
if(NOT PICO_PLATFORM)
    return()
endif()

# Generate PIO header from .pio file
pico_generate_pio_header(${PROJECT_NAME} 
    ${CMAKE_CURRENT_LIST_DIR}/pid_dma.pio
)

# Add source files
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/pid_dma_rp2040.c
)

# Add include directory
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link required pico-sdk libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_timer
    hardware_clocks
)

# Enable DMA motor driver in config
target_compile_definitions(${PROJECT_NAME} PRIVATE
    PBDRV_CONFIG_MOTOR_DRIVER_RP2040_DMA=1
)
