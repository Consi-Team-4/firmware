add_executable(${ProjectName}
    main.cpp
    freeRTOSHelpers.c
    imu/LSM6DSOX.cpp
    imu/imu.cpp
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/imu
)

target_link_libraries(${ProjectName} 
    pico_stdlib 
    FreeRTOS-Kernel-Heap4
    i2c_dma
    )

pico_add_extra_outputs(${ProjectName})