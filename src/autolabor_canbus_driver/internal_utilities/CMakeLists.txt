cmake_minimum_required(VERSION 3.13)

set(CMAKE_CXX_STANDARD 11)
project(pm1_driver_internal_utilities)

add_library(pm1_driver_internal_utilities
        big_endian_transform.h
        big_endian_transform.cpp)

add_executable(internal_test main.cpp)
target_link_libraries(internal_test pm1_driver_internal_utilities)
