cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(annotation_prep)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable (annotation_prep
  annotation_prep.cpp
  )

target_link_libraries (annotation_prep ${OpenCV_LIBS})
