cmake_minimum_required(VERSION 3.5 )
project(KalmanFilter)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -pthread")

add_executable(KalmanFilter KalmanFilter.cpp)
