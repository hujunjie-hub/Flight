cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(bmm350 C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(bmm350 main.c rtrobot_common.c bmm350/rtrobot_bmm350.c bmm350/bmm350.c )
include_directories(. bmm350)
pico_add_extra_outputs(bmm350)

#Pull in our pico_stdlib which aggregates commonly used features
target_link_libraries(bmm350 pico_stdlib hardware_i2c hardware_spi)

# enable usb output, disable uart output
pico_enable_stdio_usb(bmm350 1)
pico_enable_stdio_uart(bmm350 0)

