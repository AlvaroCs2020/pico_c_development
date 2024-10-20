# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(test C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
# Agregar biblioteca pico_arm con ArmHandler.c
#add_library(pico_arm code/ArmHandler.c)
#add_library(pico_servo servo/servo.c)
add_subdirectory(servo)
add_subdirectory(code)

#add_library(pico_arm code/ArmHandler.c)

#target_link_libraries(Tutorial PUBLIC MathFunctions)
add_executable(test test.c code/ArmHandler.c servo/servo.c)

pico_set_program_name(test "test")
pico_set_program_version(test "0.1")

# Generate PIO header
pico_generate_pio_header(test ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(test 1)
pico_enable_stdio_usb(test 1)

#
target_link_libraries(test
        pico_stdlib
#        pico_servo
#        pico_arm
        hardware_pwm
        hardware_clocks 
    )
# Add the standard include files to the build
target_include_directories(test PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
#    pico_servo PUBLIC ${CMAKE_CURRENT_LIST_DIR}
    #pico_arm PUBLIC ${CMAKE_CURRENT_LIST_DIR}/code
  )

# Add any user requested libraries
target_link_libraries(test 
        hardware_spi
        hardware_pio
        )

pico_add_extra_outputs(test)

