cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(label_point_cloud)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (label_point_cloud label_point_cloud.cpp)
target_link_libraries (label_point_cloud ${PCL_LIBRARIES} ${OpenCV_LIBS})
