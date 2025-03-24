add_executable(${ProjectName}
    encoder/simple_encoder_substep.c
    encoder/encoder.c
    imu/imu.c
    debug.c
    freeRTOSHelpers.c
    main.c
    servo.c
    esc_pid.c
    drive_esc.c
)

pico_generate_pio_header(${ProjectName}
    ${CMAKE_CURRENT_LIST_DIR}/encoder/simple_encoder_substep.pio
)


target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/encoder
    ${CMAKE_CURRENT_LIST_DIR}/imu
    ${CMAKE_CURRENT_LIST_DIR}/lidar
)

target_link_libraries(${ProjectName} 
    pico_stdlib
    hardware_pwm
    hardware_pio
    FreeRTOS-Kernel-Heap4
    i2c_dma
    ekf_engl
    )

# Enable Stdio over usb
pico_enable_stdio_uart(${ProjectName} 0)
pico_enable_stdio_usb(${ProjectName} 1)

# Needed to generate the UF2 or elf or something like that.
pico_add_extra_outputs(${ProjectName})
