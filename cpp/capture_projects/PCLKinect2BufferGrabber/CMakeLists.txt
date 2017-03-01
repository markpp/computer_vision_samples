# CMake 
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Solution
project(solution)

# Non Default Install
#set(PCL_DIR "$ENV{PCL_ROOT}/cmake" CACHE FILEPATH "The directory containing a CMake configuration file for PCL." )

# Find Package
find_package(PCL 1.7 REQUIRED)

# Additional Include Directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories($ENV{KINECTSDK20_DIR}/inc)

# Additional Library Directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories($ENV{KINECTSDK20_DIR}/Lib/$(PlatformTarget))

# Preprocessor Definitions
add_definitions(${PCL_DEFINITIONS})

# Project
set(HEADER kinect2_grabber.h stdafx.h)
set(SOURCE sample.cpp stdafx.cpp)
add_executable(project ${HEADER} ${SOURCE})

# Additional Dependencies
target_link_libraries(project ${PCL_LIBRARIES})
target_link_libraries(project Kinect20.lib)