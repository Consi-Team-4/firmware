add_library(ekf_engl INTERFACE)

target_include_directories(ekf_engl INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_sources(ekf_engl INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ekf.cpp
    ${CMAKE_CURRENT_LIST_DIR}/matrix.cpp
)

target_link_libraries(ekf_engl INTERFACE
    pico_stdlib
    pico_runtime
)