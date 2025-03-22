cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(ActiveSuspensionRC C CXX ASM)

pico_sdk_init()

add_executable(${ProjectName}
    main.c
    # Sensors
    src/sensors/imu.c
    src/sensors/lidar.c
    src/sensors/encoder.c
    # Controllers
    src/controllers/speed_controller.c
    src/controllers/suspension_controller.c
    src/controllers/pid.c
    # Tasks
    src/tasks/imu_task.c
    src/tasks/wifi_task.c
    src/tasks/lidar_task.c
    src/tasks/debug_task.c
    src/tasks/task_common.c
    # Utils
    src/utils/FreeRTOSHelpers.c
    src/utils/servo.c
    src/utils/kalman.cpp
)

# If you use PIO code from encoder folder
pico_generate_pio_header(${ProjectName}
    ${CMAKE_CURRENT_LIST_DIR}/encoder/simple_encoder_substep.pio
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src/sensors
    ${CMAKE_CURRENT_LIST_DIR}/src/controllers
    ${CMAKE_CURRENT_LIST_DIR}/src/tasks
    ${CMAKE_CURRENT_LIST_DIR}/src/utils
)

target_link_libraries(${ProjectName} 
    pico_stdlib
    hardware_pwm
    hardware_pio
    hardware_dma
    hardware_i2c
    hardware_uart
    FreeRTOS-Kernel-Heap4
    i2c_dma
    ukf_engl
)

pico_enable_stdio_uart(${ProjectName} 0)
pico_enable_stdio_usb(${ProjectName} 1)
pico_add_extra_outputs(${ProjectName})