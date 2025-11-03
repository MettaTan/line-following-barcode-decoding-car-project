cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(week6_drivers C CXX ASM)
pico_sdk_init()

# ===== encoder_demo =====
add_executable(encoder_demo
    main_encoder_test.c
    encoder.c
)
target_include_directories(encoder_demo PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_compile_definitions(encoder_demo PRIVATE
    ENCODER_GPIO=2
    SLOTS_PER_REV=20
)
target_link_libraries(encoder_demo pico_stdlib)
pico_enable_stdio_usb(encoder_demo 1)
pico_enable_stdio_uart(encoder_demo 0)
pico_add_extra_outputs(encoder_demo)

# ===== week6_demo =====
add_executable(week6_demo
    main.c
    encoder.c
)
target_include_directories(week6_demo PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(week6_demo pico_stdlib hardware_adc hardware_pwm)
pico_enable_stdio_usb(week6_demo 1)
pico_enable_stdio_uart(week6_demo 0)
pico_add_extra_outputs(week6_demo)
