cmake_minimum_required(VERSION 3.13)

# Board configuration
if(NOT DEFINED TARGET_BOARD)
    set(TARGET_BOARD "picow")
endif()

if(TARGET_BOARD STREQUAL "picow")
    set(PICO_BOARD pico_w)
    set(NUM_MOTORS 4)
    set(IS_PICO_2W 0)
    set(OUTPUT_NAME pybricks_picow)
elseif(TARGET_BOARD STREQUAL "pico2w")
    set(PICO_BOARD pico2_w)
    set(NUM_MOTORS 12)
    set(IS_PICO_2W 1)
    set(OUTPUT_NAME pybricks_pico2w)
else()
    message(FATAL_ERROR "Unknown TARGET_BOARD: ${TARGET_BOARD}")
endif()

# WiFi mode
if(NOT DEFINED WIFI_MODE)
    set(WIFI_MODE "THREADSAFE_BACKGROUND")
endif()

if(WIFI_MODE STREQUAL "POLL")
    set(CYW43_ARCH_LIB pico_cyw43_arch_lwip_poll)
    add_compile_definitions(WIFI_MODE_POLL=1)
elseif(WIFI_MODE STREQUAL "THREADSAFE_BACKGROUND")
    set(CYW43_ARCH_LIB pico_cyw43_arch_lwip_threadsafe_background)
    add_compile_definitions(WIFI_MODE_THREADSAFE_BACKGROUND=1)
elseif(WIFI_MODE STREQUAL "SYS")
    set(CYW43_ARCH_LIB pico_cyw43_arch_lwip_sys_freertos)
    add_compile_definitions(WIFI_MODE_SYS=1)
else()
    message(FATAL_ERROR "Unknown WIFI_MODE")
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set encryption key for OTA (CHANGE THIS!)
if(NOT DEFINED PFB_AES_KEY)
    set(PFB_AES_KEY "0123456789ABCDEF0123456789ABCDEF")
    message(WARNING "Using default AES key - change this for production!")
endif()

project(pybricks C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Add pico_fota_bootloader only for picow (not pico2w)
if(TARGET_BOARD STREQUAL "picow")
    add_subdirectory(pico_fota_bootloader)
    set(ENABLE_OTA TRUE)
else()
    set(ENABLE_OTA FALSE)
endif()

# Common paths
set(PBIO_TOP ${CMAKE_SOURCE_DIR}/../../lib/pbio)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${PBIO_TOP}/platform/pico_common
    ${PBIO_TOP}/include
    ${PBIO_TOP}/drv/imu
    ${PBIO_TOP}/drv/battery
    ${PBIO_TOP}/drv/watchdog
    ${PBIO_TOP}/drv/pid_dma
    ${PBIO_TOP}/drv/ioport
    ${PBIO_TOP}/drv/storage
    ${PBIO_TOP}/drv/motion_dsp
)

if(IS_PICO_2W)
    # Don't modify compiler flags - just add defines
    add_compile_definitions(
        ARM_MATH_CM33
        ARM_MATH_DSP
        __ARM_FEATURE_DSP=1
    )
    
    message(STATUS "RP2350 DSP support enabled (via defines)")
endif()

add_compile_definitions(CYW43_ENABLE_BLUETOOTH=1)

if(ENABLE_OTA)
    set(OTA_SOURCES ota_handler.c)
else()
    set(OTA_SOURCES)
endif()

add_executable(${OUTPUT_NAME}
    main.c
    system_test.c
    btstack_port.c
    httpd_content.c
    ${OTA_SOURCES}
    ${PBIO_TOP}/platform/pico_common/platform.c
    ${PBIO_TOP}/drv/pid_dma/pid_dma_rp2040.c
    ${PBIO_TOP}/drv/imu/imu_bno085.c
    ${PBIO_TOP}/drv/watchdog/watchdog_rp2040.c
    ${PBIO_TOP}/drv/battery/battery_rp2040_adc.c
    ${PBIO_TOP}/drv/ioport/ioport_rp2040.c
    ${PBIO_TOP}/drv/storage/storage_flash_rp2040.c
    ${PBIO_TOP}/drv/storage/storage_sd_rp2040.c
    ${PBIO_TOP}/drv/storage/storage.c
)

if(IS_PICO_2W)
    list(APPEND PBIO_SOURCES
        ${PBIO_TOP}/drv/motion_dsp/dsp_verification.c
        ${PBIO_TOP}/drv/motion_dsp/encoder_filter_dsp.c
        ${PBIO_TOP}/drv/motion_dsp/encoder_filter_test.c
        ${PBIO_TOP}/drv/motion_dsp/pid_controller_dsp.c
        ${PBIO_TOP}/drv/motion_dsp/pid_controller_test.c
        ${PBIO_TOP}/drv/motion_dsp/trajectory_planner_dsp.c
    )
endif()


pico_generate_pio_header(${OUTPUT_NAME} ${PBIO_TOP}/drv/pid_dma/pid_dma.pio)

target_include_directories(${OUTPUT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(${OUTPUT_NAME}
    pico_stdlib ${CYW43_ARCH_LIB}
    pico_btstack_ble
    pico_btstack_cyw43
    pico_lwip_http
    pico_lwip_sntp
    hardware_pio
    hardware_dma
    hardware_pwm
    hardware_adc
    hardware_i2c
    hardware_spi
    hardware_flash
    pico_multicore
)

# Link OTA bootloader library only for picow
if(ENABLE_OTA)
    target_link_libraries(${OUTPUT_NAME} pico_fota_bootloader_lib)
    pfb_compile_with_bootloader(${OUTPUT_NAME})
endif()

pico_add_extra_outputs(${OUTPUT_NAME})

pico_enable_stdio_usb(${OUTPUT_NAME} 1)
pico_enable_stdio_uart(${OUTPUT_NAME} 0)
