# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

project(op5-gyroscope)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_path(WIRINGPI_INCLUDE_DIRS NAMES wiringPi.h)

add_executable(op5-gyroscope main.cpp DFRobot.cpp)

target_link_libraries( op5-gyroscope ${WIRINGPI_LIBRARIES})