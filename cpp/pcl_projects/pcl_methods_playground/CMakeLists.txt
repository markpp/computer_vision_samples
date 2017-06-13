cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

project(PCLSegmentMulti)

find_package(PCL 1.7.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PCLSegmentMulti
                PCLSegmentMulti.cpp
                src/Segmentation/RGS.cpp
                src/Segmentation/CRGS.cpp
                src/Segmentation/SVC.cpp
                src/Segmentation/LCCP.cpp
                src/Segmentation/CPC.cpp
                src/Conversion/Conversion.cpp
                src/Conversion/NormalMap.cpp
                src/Filters/PassThrough.cpp
                src/Filters/Smoothing.cpp
                src/Presentation/ColorCoding/DepthCoding.cpp
                src/Presentation/ColorCoding/NormalCoding.cpp
                src/Presentation/Normals.cpp
                src/Presentation/PrincipalCurves.cpp
                src/Presentation/Settings.cpp
                src/Presentation/Graphics.cpp
                src/Registration/NDT.cpp
                src/Registration/ICP.cpp)

target_link_libraries (PCLSegmentMulti ${PCL_LIBRARIES} ${OpenCV_LIBS})
