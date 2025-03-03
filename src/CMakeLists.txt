add_executable(${ProjectName}
    main.c
    freeRTOSHelpers.c
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${ProjectName} 
    pico_stdlib 
    FreeRTOS-Kernel
    )

pico_add_extra_outputs(${ProjectName})