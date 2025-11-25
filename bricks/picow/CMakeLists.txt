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

project(${OUTPUT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(PBTOP ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PBIO_TOP ${PBTOP}/lib/pbio)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PBIO_TOP}/platform/pico_common
    ${PBIO_TOP}/include
    ${PBTOP}/lib
    ${PBIO_TOP}/drv/pid_dma
    ${PBIO_TOP}/drv/watchdog
    ${PBIO_TOP}/drv/battery
    ${PBIO_TOP}/drv/ioport
    ${PBIO_TOP}/drv/storage
)

if(IS_PICO_2W)
    add_compile_definitions(PICO_2W=1)
endif()
add_compile_definitions(CYW43_ENABLE_BLUETOOTH=1)

add_executable(${OUTPUT_NAME}
    main.c
    system_test.c
    btstack_port.c
    httpd_content.c
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

pico_generate_pio_header(${OUTPUT_NAME} ${PBIO_TOP}/drv/pid_dma/pid_dma.pio)

target_include_directories(${OUTPUT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(${OUTPUT_NAME}
    pico_stdlib ${CYW43_ARCH_LIB}
    pico_btstack_ble pico_btstack_cyw43
    pico_lwip_http pico_lwip_sntp pico_multicore
    hardware_adc hardware_i2c hardware_uart hardware_spi
    hardware_flash hardware_sync hardware_pwm hardware_pio hardware_dma
)

pico_add_extra_outputs(${OUTPUT_NAME})

message(STATUS "Target: ${TARGET_BOARD}, WiFi: ${WIFI_MODE}, Motors: ${NUM_MOTORS}")
