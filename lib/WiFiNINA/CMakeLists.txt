cmake_minimum_required(VERSION 3.13)

project(WiFiNINA C CXX ASM)

# Optional: Explicit C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Add the src folder which contains the WiFiNINA library
add_subdirectory(src)