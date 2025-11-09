cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(barcode_test C CXX ASM)
pico_sdk_init()

add_executable(week10_integration
    main.c          # <-- this barcode test file
)

target_link_libraries(week10_integration
    pico_stdlib
    hardware_gpio
)

pico_enable_stdio_usb(week10_integration 1)
pico_enable_stdio_uart(week10_integration 0)

pico_add_extra_outputs(week10_integration)
