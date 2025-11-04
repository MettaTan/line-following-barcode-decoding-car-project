cmake_minimum_required(VERSION 3.13)

# Point CMake at the Pico SDK
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(week10_integration C CXX ASM)
pico_sdk_init()

# === Executable ===
add_executable(week10_integration
    main.c
    encoder.c
    line.c
    motor.c
    imu.c
)

# === Includes ===
target_include_directories(week10_integration PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# === Libraries ===
# (GPIO, PWM, ADC, I2C, timer, irq are required by your drivers)
target_link_libraries(week10_integration
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_adc
    hardware_i2c
    hardware_timer
    hardware_irq
)

# === USB stdio ===
pico_enable_stdio_usb(week10_integration 1)
pico_enable_stdio_uart(week10_integration 0)

# === UF2 output ===
pico_add_extra_outputs(week10_integration)
