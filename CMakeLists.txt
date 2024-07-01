# FOR mouse_imu.c!!!!!!!!

cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(MouseProject)

# Initialize the Raspberry Pi Pico SDK.
pico_sdk_init()

# Source files.
add_executable(mouse_imu
  mouse_imu.c
)

# Pull in dependencies
target_link_libraries(mouse_imu pico_stdlib hardware_spi pico_stdio_usb hardware_pwm)
#pico_multicore hardware_dma hardware_adc hardware_pio)

# Create map/bin/hex/uf2 file etc.
#pico_add_extra_outputs(mouse_imu)

# Enable usb output, disable uart output.
#pico_enable_stdio_usb(mouse_imu 1)
#pico_enable_stdio_uart(mouse_imu 0)
