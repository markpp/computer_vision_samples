cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(find_plane)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (find_plane find_plane.cpp)
target_link_libraries (find_plane ${PCL_LIBRARIES})
