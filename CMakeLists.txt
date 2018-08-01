cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(OpenCV_DIR "/home/zk8888/opencv-3.1.0/release")

project(jenga)

set(CMAKE_CXX_FLAGS "-std=c++11")

# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
#find_package(OpenMesh REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories("/usr/include/libusb-1.0/")
include_directories(${freenect2_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OpenMesh_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${OpenMesh_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${OpenCV_DEFINITIONS})
add_definitions(${OpenMesh_DEFINITIONS})

add_executable (jenga src/main.cpp src/jenga_game.cpp
        include/jenga_game.hpp src/mesh_sampling.cpp include/k2g.hpp)
target_link_libraries (jenga ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${BOOST_LIBS} OpenMeshCore
        OpenMeshTools)
