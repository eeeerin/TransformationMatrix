cmake_minimum_required(VERSION 3.9)
project(PA2TransformationMatrix)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE "Release")

include_directories("/usr/include/eigen3")

add_executable(PA2TransformationMatrix main.cpp)