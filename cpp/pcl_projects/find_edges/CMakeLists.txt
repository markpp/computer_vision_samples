cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(find_edges)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (find_edges find_edges.cpp)
target_link_libraries (find_edges ${PCL_LIBRARIES})
