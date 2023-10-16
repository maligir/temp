cmake_minimum_required(VERSION 2.8)
project(k4a_borrowed)

message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

set(LIB_K4A_BORROWED_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

include_directories(${LIB_K4A_BORROWED_INCLUDE_DIRS})

#set(CMAKE_BUILD_TYPE Debug)
#find_package(Eigen3 REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(Cairo REQUIRED)
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

find_library(KINECT_INTERFACE_LIB kinectinterface)

SET(K4A_BORROWED_SRC
    src/gpudepthtopointcloudconverter.cpp
    src/kfr_depth_to_pointcloud_gpu.cpp
)

SET(K4A_BORROWED_INCLUDES
    gpudepthtopointcloudconverter.h
    openglhelpers.h
    kfr_depth_to_pointcloud_gpu.h
)

include_directories(
    #${GTK3_INCLUDE_DIRS}
    #${CAIRO_INCLUDE_DIRS}
    #${VISION_GEOMETRY_INCLUDE_DIRS}
    #${APRIL_TAG_INCLUDE_DIRS}
    #${EIGEN3_INCLUDE_DIR}
    ${LIB_KINECT_INTERFACE_INCLUDE_DIRS}
)

add_library(k4a_borrowed ${K4A_BORROWED_SRC})