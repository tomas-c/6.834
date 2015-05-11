cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(crrt)

INCLUDE_DIRECTORIES(
    include/
)

add_library (CRRT src/RRT.cpp src/RRT.h)
add_subdirectory(tests)