cmake_minimum_required(VERSION 3.13)
cmake_policy(SET CMP0079 NEW)

project(firmware C CXX ASM)

add_executable(firmware
    encoder/encoder.c
    encoder/simple_encoder_substep.c
    debug.c
    freeRTOSHelpers.c
    main.c
    servo.c
    sensors/imu.c
    sensors/lidar.c
    tasks/debug_task.c
    tasks/wifi_task.c
    controllers/speed_controller.c
    controllers/suspension_controller.c
)

# Generate the PIO header
pico_generate_pio_header(firmware
    ${CMAKE_CURRENT_LIST_DIR}/encoder/simple_encoder_substep.pio
)

target_include_directories(firmware PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/tasks
    ${CMAKE_CURRENT_LIST_DIR}/sensors
    ${CMAKE_CURRENT_LIST_DIR}/controllers
)

target_link_libraries(firmware 
    pico_stdlib
    hardware_pwm
    hardware_pio
    hardware_dma
    hardware_i2c
    hardware_uart
    FreeRTOS-Kernel-Heap4
    i2c_dma
    ukf_engl
    WiFiNINA
    arduino_core
)

# Enable Stdio over USB and disable UART
pico_enable_stdio_uart(firmware 0)
pico_enable_stdio_usb(firmware 1)

# Add UF2 and binary outputs
pico_add_extra_outputs(firmware)