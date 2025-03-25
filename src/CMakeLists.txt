add_executable(${ProjectName}
    # Core
    main.c

    # Controllers
    controllers/esc.c
    controllers/encoder.c
    controllers/servo.c
    controllers/simple_encoder_substep.c

    # Sensors
    sensors/imu.c
    sensors/lidar.c

    # Tasks: Active components
    tasks/heartbeat.c


    # Utilities: Passive helper functions 
    utils/log.c
    utils/debug.c
    utils/freeRTOSHelpers.c
    utils/kalman.cpp
)

# PIO header generation (adjust if you're using more .pio files)
pico_generate_pio_header(${ProjectName}
    ${CMAKE_CURRENT_LIST_DIR}/controllers/simple_encoder_substep.pio
)

# Include directories
target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/controllers
    ${CMAKE_CURRENT_LIST_DIR}/sensors
    ${CMAKE_CURRENT_LIST_DIR}/tasks
    ${CMAKE_CURRENT_LIST_DIR}/utils
)

# Linked libraries
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

# Enable USB and UART stdio
pico_enable_stdio_uart(${ProjectName} 0)
pico_enable_stdio_usb(${ProjectName} 1)

# UF2 and ELF output
pico_add_extra_outputs(${ProjectName})