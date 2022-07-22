cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)
project(picobno)
pico_sdk_init()
add_executable(picobno picobno.c)
target_link_libraries(picobno 
    pico_stdlib
    hardware_i2c
)
pico_enable_stdio_usb(picobno 1)
pico_enable_stdio_uart(picobno 0)
pico_add_extra_outputs(picobno)