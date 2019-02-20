#=============================================================================#
# Author: Gregory DEPUILLE                                                    #
# Date:   13/05/2016                                                          #
# Description: Arduino CMake Robot Nerell                                     #
#=============================================================================#

# Contrôle de la version minimale de CMake
cmake_minimum_required(VERSION 3.8.2)

# Pas d'upload par défaut
if (NOT DEFINED ARDUINO_UPLOAD)
    set(ARDUINO_UPLOAD false)
endif()

# Définition de la tool chain de build
set(CMAKE_TOOLCHAIN_FILE ../arduino-cmake-ng/cmake/Arduino-Toolchain.cmake)

# Compilateur flags
#set(ARDUINO_C_FLAGS      "-ffunction-sections -fdata-sections")
#set(ARDUINO_CXX_FLAGS    "${ARDUINO_C_FLAGS} -fno-exceptions")
#set(ARDUINO_LINKER_FLAGS "-Wl,--gc-sections")

set(ARDUINO_CXX_FLAGS    "-std=c++11")

#add_definitions(-DDEBUG_MODE)

#====================================================================#
#  Setup Project                                                     #
#====================================================================#

project(NerellRobot C CXX ASM)

# Récupération de la board
get_board_id(board_id mega atmega2560)

# RobotSystemLib
file(GLOB_RECURSE LIB_SOURCES "../robot-system-lib/src/*.cpp" "../robot-system-lib/src/*.c")
file(GLOB_RECURSE LIB_HEADERS "../robot-system-lib/src/*.h")
add_library(RobotSystem STATIC ${LIB_SOURCES} ${LIB_HEADERS})
target_include_directories(RobotSystem PUBLIC ../robot-system-lib/src)

# Wire Lib
find_library_header_files("${ARDUINO_CMAKE_PLATFORM_LIBRARIES_PATH}/Wire/src" wire_lib_headers)
find_library_source_files("${ARDUINO_CMAKE_PLATFORM_LIBRARIES_PATH}/Wire/src" wire_lib_source_files)
set(wire_lib_sources ${wire_lib_headers} ${wire_lib_source_files})
target_include_directories(RobotSystem PUBLIC ${ARDUINO_CMAKE_PLATFORM_LIBRARIES_PATH}/Wire/src)

# Build
add_arduino_executable(NerellRobot ${board_id} src/main.cpp src/define.h)
link_platform_library(NerellRobot Wire ${board_id})
link_arduino_library(NerellRobot RobotSystem ${board_id})

# Upload
if (ARDUINO_UPLOAD)
    message("-- Upload enabled")
    upload_arduino_target(NerellRobot "${board_id}" /dev/ttyUSB0)
else()
    message("-- Upload disabled")
endif()
