add_library(WiFiNINA STATIC
    WiFi.cpp
    WiFiClient.cpp
    WiFiSSLClient.cpp
    WiFiServer.cpp
    WiFiStorage.cpp
    WiFiUdp.cpp
    utility/WiFiSocketBuffer.cpp
    utility/nano_rp2040_support.cpp
    utility/server_drv.cpp
    utility/spi_drv.cpp
    utility/wifi_drv.cpp
)

target_include_directories(WiFiNINA PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/utility
)

target_link_libraries(WiFiNINA PUBLIC arduino_core)

# Enforce C++11 standard
target_compile_features(WiFiNINA PUBLIC cxx_std_11)