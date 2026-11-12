cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME John_Deere_Tractor)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/Control/math_utils.c
    App/Control/pid.c
    App/Actuators/motor.c
    App/Actuators/servo.c
    App/Control/heading_hold.c
    App/Sensors/bno085.c
    App/Sensors/sh2/sh2.c
    App/Sensors/sh2/sh2_SensorValue.c
    App/Sensors/sh2/sh2_util.c
    App/Sensors/sh2/shtp.c
    App/Sensors/distance.c
    App/Sensors/tof.c
    App/Sensors/vl53l1x_uld/VL53L1X_api.c
    App/Sensors/vl53l1x_uld/vl53l1_platform.c
    # Ultrasonic sensor: not used at present.
    # App/Sensors/hcsr04.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App/Sensors
    App/Control
    App/Actuators
    App/StateMachine
    App/Config
    App/Sensors/sh2
    App/Sensors/vl53l1x_uld
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
