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
set(CMAKE_PROJECT_NAME TwoWheelBalcance)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        MPU6050/Src/MPU6050.cpp
        MPU6050/Inc/MPU6050.h
        BSP/Inc/bsp_dwt.h
        BSP/Src/bsp_dwt.cpp
        Debug_Vars/debug_vars.cpp
        Debug_Vars/debug_vars.h
        task/TaskBase.h
        task/Inc/MPU6050_read.h
        task/Src/MPU6050_read.cpp
        Algorithm/Inc/CRC8.h
        Algorithm/Src/CRC8.cpp
        Algorithm/Src/CRC16.cpp
        Algorithm/Inc/CRC16.h
        Algorithm/Inc/FusionAHRS.h
        Algorithm/Inc/MahonyAHRS.h
        Algorithm/Src/FusionAHRS.cpp
        Algorithm/Src/MahonyAHRS.c
        Modules/Src/tcrt.cpp
        Modules/Inc/tcrt.h
        task/Src/Track_Task.cpp
        task/Inc/Track_Task.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        MPU6050
        MPU6050/Inc
        MPU6050/Src
        BSP
        BSP/Inc
        BSP/Src
        Debug_Vars
        task
        task/Inc
        task/Src
        Serial
        Serial/Inc
        Serial/Src
        Algorithm
        Algorithm/Src
        Algorithm/Inc
        Modules
        Modules/Src
        Modules/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
