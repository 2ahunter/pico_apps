cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(AHRS C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()


add_executable(AHRS ahrs_main.c BNO055.c ahrs_m_update.c )

pico_enable_stdio_usb(AHRS 1)
pico_enable_stdio_uart(AHRS 1)
pico_add_extra_outputs(AHRS)

target_link_libraries(AHRS pico_stdlib hardware_i2c)

