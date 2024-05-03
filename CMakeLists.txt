# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

project(op5-gyroscope)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_path(WIRINGPI_INCLUDE_DIRS NAMES wiringPi.h)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(op5-gyroscope main.cpp i2c.c madjvic_filter.cpp gyro_base.cpp gyro.h DFRobot.cpp Kalman.cpp DFRobot.h madjvic_filter.h)

target_link_libraries( op5-gyroscope ${WIRINGPI_LIBRARIES} ${OpenCV_LIBS} pthread)