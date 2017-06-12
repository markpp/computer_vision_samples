cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

project(PCLSuperVoxelSegmentation)

find_package(PCL 1.7.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PCLSuperVoxelSegmentation
  PCLSuperVoxelSegmentation.cpp
  src/Settings.cpp
  src/SVC.cpp
  src/Graphics.cpp
  src/Filters.cpp
  src/FeatureExtractor.cpp
  src/Normals.cpp
  )

target_link_libraries (PCLSuperVoxelSegmentation ${PCL_LIBRARIES} ${OpenCV_LIBS})
