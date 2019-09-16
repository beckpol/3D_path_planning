cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kinect)

# Use cmake -DCMAKE_BUILD_TYPE=Release ..     for release version
SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -pipe")

# Find PCL,QT4 and VTK
find_package(PCL 1.7 REQUIRED)			
find_package(Qt4 REQUIRED)
find_package(VTK)

# Include all the needed library
include_directories(${PCL_INCLUDE_DIRS} ${QT_INCLUDE_DIRS})
include(${QT_USE_FILE})
include(${VTK_USE_FILE})
include_directories (${CMAKE_BINARY_DIR})

link_directories(${PCL_LIBRARY_DIRS} ${QT_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${QT_DEFINITIONS})

# Set sources, headers and uis
SET(kinect_SOURCES main.cpp mainwindow.cpp)
SET(kinect_UIS mainwindow.ui)
SET(kinect_HEADERS mainwindow.h)

# Wrap them together
QT4_WRAP_UI(kinect_SOURCES_UIS_H ${kinect_UIS})
QT4_WRAP_CPP(kinect_HEADERS_MOC ${kinect_HEADERS})

# Compile
add_executable (Kinect ${kinect_SOURCES} ${kinect_HEADERS_MOC} ${kinect_SOURCES_UIS_H})
target_link_libraries (Kinect QVTK ${PCL_LIBRARIES} ${QT_LIBRARIES}) 

