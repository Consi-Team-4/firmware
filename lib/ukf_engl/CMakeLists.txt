add_library(ukf_engl INTERFACE)

target_include_directories(ukf_engl INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_sources(ukf_engl INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ukf.cpp
    ${CMAKE_CURRENT_LIST_DIR}/matrix.cpp
)

target_link_libraries(ukf_engl INTERFACE
    pico_stdlib
    pico_runtime
)