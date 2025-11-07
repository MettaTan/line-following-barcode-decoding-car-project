cmake_minimum_required(VERSION 3.13)

# --- MUST be before project(): tells SDK we are building for Pico W (CYW43 available)
set(PICO_BOARD pico_w)

# Point to the Pico SDK
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(week10_integration C CXX ASM)
pico_sdk_init()

add_executable(week10_integration
    main.c
    encoder.c
    line.c
    motor.c
    imu.c
    mqtt_pico.c     # your Wi-Fi/MQTT shim
)

target_include_directories(week10_integration PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# (Optional) define SSID/PASS here if not in a header
# target_compile_definitions(week10_integration PRIVATE
#     WIFI_SSID="\"YourSSID\"" WIFI_PASS="\"YourPassword\""
# )

target_link_libraries(week10_integration
    pico_stdlib

    # Wi-Fi + lwIP + MQTT
    pico_lwip_mqtt
    pico_cyw43_arch_lwip_threadsafe_background

    # Peripherals you use
    hardware_gpio
    hardware_pwm
    hardware_adc
    hardware_i2c
    hardware_timer
    hardware_irq
)

# stdio config
pico_enable_stdio_usb(week10_integration 1)
pico_enable_stdio_uart(week10_integration 0)

# UF2, map/bin/hex
pico_add_extra_outputs(week10_integration)
