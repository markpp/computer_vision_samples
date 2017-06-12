cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rotate_point_cloud)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (rotate_point_cloud rotate_point_cloud.cpp)
target_link_libraries (rotate_point_cloud ${PCL_LIBRARIES})
