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

set(PICO_BOARD arduino_nano_rp2040_connect CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.12)

# Name the project
set(ProjectName firmware)

# Set FreeRTOS kernel path
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)

# Import Pico SDK and FreeRTOS
include(pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Define project
project(${ProjectName} C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

# Add subdirectories
add_subdirectory(src)

# External libs
add_subdirectory(lib/pico-i2c-dma/src)
add_subdirectory(lib/ukf_engl)

# Optional: Add WiFiNINA if needed in the future
# add_subdirectory(lib/WiFiNINA/src)