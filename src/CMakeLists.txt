add_executable(${ProjectName}
    main.c
    freeRTOSHelpers.c
    debug.c
    imu/imu.c
    servo.c
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/imu
)

target_link_libraries(${ProjectName} 
    pico_stdlib 
    hardware_pwm # Not sure why this one needs to be specified separately
    FreeRTOS-Kernel-Heap4
    i2c_dma
    )

# Enable Stdio over usb
pico_enable_stdio_uart(${ProjectName} 0)
pico_enable_stdio_usb(${ProjectName} 1)

# Needed to generate the UF2 or elf or something like that.
pico_add_extra_outputs(${ProjectName})
