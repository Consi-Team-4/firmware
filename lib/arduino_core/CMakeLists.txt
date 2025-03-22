add_library(arduino_core STATIC
    IPAddress.cpp
    Print.cpp
    Stream.cpp
    WString.cpp
    WMath.cpp
)

target_include_directories(arduino_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/avr  # if pgmspace.h lives in an avr subfolder
)

target_compile_features(arduino_core PUBLIC cxx_std_11)