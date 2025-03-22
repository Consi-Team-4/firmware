# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)

# Set CMake policy to avoid cross-directory link issues
cmake_policy(SET CMP0079 NEW)

# Set target board for Pico SDK
set(PICO_BOARD arduino_nano_rp2040_connect CACHE STRING "Board type")

# Project Name
set(ProjectName firmware)
project(${ProjectName} C CXX ASM)

# Set any variables required for importing libraries
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)

# Import Pico SDK and FreeRTOS-Kernel
include(pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Initialize Pico SDK
pico_sdk_init()

# Add subdirectories for your libraries
add_subdirectory(lib/arduino_core)
add_subdirectory(lib/WiFiNINA)
add_subdirectory(lib/pico-i2c-dma/src)
add_subdirectory(lib/ukf_engl)

# Add the firmware source code directory
add_subdirectory(src)

# NOTE:
# WiFiNINA links arduino_core INSIDE its own CMakeLists.txt now.
# Do NOT call target_link_libraries(WiFiNINA PUBLIC arduino_core) here.