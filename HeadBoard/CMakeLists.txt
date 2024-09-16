cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

#include(${PICO_SDK_PATH}/lib/tinyusb/hw/bsp/family_support.cmake)

project(MouseImuProject)

# Initialize the Raspberry Pi Pico SDK.
pico_sdk_init()

# Source files.
add_executable(main
  main.c
)

# Pull in dependencies
target_link_libraries(main PUBLIC
  pico_stdlib hardware_spi hardware_pwm hardware_uart # Main logic libraries.
)

# Makes local #includes work.
target_include_directories(main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create map/bin/hex/uf2 file etc.
#pico_add_extra_outputs(mouse_imu)

# Enable usb output, disable uart output.
#pico_enable_stdio_usb(main 1)
#pico_enable_stdio_uart(main 0)
