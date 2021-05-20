cmake_minimum_required(VERSION 3.19)
project(geo1101)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.3 REQUIRED COMPONENTS common io filters
        visualization surface features kdtree)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(geo1101 main.cpp)
target_link_libraries (geo1101 ${PCL_LIBRARIES})
