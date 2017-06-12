cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCLSimpleVisualizer_filter)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PCLSimpleVisualizer_filter PCLSimpleVisualizer_filter.cpp src/filters.cpp)
target_link_libraries (PCLSimpleVisualizer_filter ${PCL_LIBRARIES})
