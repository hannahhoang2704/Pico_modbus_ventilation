# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set board type because we are building for PicoW
set(PICO_BOARD pico_w)

# Set name of project (as PROJECT_NAME) and C/C   standards
project(uart_irq C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
)

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME}
        main.cpp
        nanomodbus.h
        nanomodbus.c
        PicoUart.cpp
        PicoUart.h
        RingBuffer.cpp
        RingBuffer.h
        Countdown.cpp
        Countdown.h
        IPStack.cpp
        IPStack.h
        lwipopts.h
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# this adds a post build command that tries to copy uf2 to Pico
#add_custom_command(TARGET ${PROJECT_NAME}
#        POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.uf2 d: )

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
        pico_stdlib
        hardware_pwm
        hardware_gpio
        pico_cyw43_arch_lwip_poll
)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)