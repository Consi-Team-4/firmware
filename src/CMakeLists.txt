add_executable(${ProjectName}
    # Core
    main.c

    # Controllers
    controller.c
    servo.c

    # Sensors
    encoder.c
    simple_encoder_substep.c
    imu.c
    lidar.c
    bluetooth.c

    # Tasks: Active components
    console.c
    heartbeat.c


    # Utilities: Passive helper functions 
    log.c
    debug.c
    freeRTOSHelpers.c

    
)

# PIO header generation (adjust if you're using more .pio files)
pico_generate_pio_header(${ProjectName}
    ${CMAKE_CURRENT_LIST_DIR}/simple_encoder_substep.pio
    ${CMAKE_CURRENT_LIST_DIR}/uart_rx.pio
)

# Include directories
target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
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
)

# Enable USB and UART stdio
pico_enable_stdio_uart(${ProjectName} 0)
pico_enable_stdio_usb(${ProjectName} 1)

# UF2 and ELF output
pico_add_extra_outputs(${ProjectName})