#=============================================================================#
# Author: Gregory DEPUILLE                                                    #
# Date:   13/05/2016                                                          #
# Description: Arduino CMake Robot Nerell                                     #
#=============================================================================#

# Inclusions des .h de la lib
include_directories(../robot-system-lib/src)

# Génération du programme par défaut
generate_arduino_firmware(NerellMega
    SRCS src/main.cpp
    HDRS src/define.h
    LIBS RobotMega
    BOARD mega2560
)