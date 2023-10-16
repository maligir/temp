cmake_minimum_required(VERSION 2.8)
project(kinect_interface)

message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

set(LIB_KINECT_INTERFACE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/azure_kinect_interface
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

include_directories(${LIB_KINECT_INTERFACE_INCLUDE_DIRS})

#set(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

SET(AZURE_KINECT_INTERFACE_SRC
    src/kinect_packet.cpp
    src/kinect_wrapper.cpp
    src/kinect_playback.cpp
    src/kfr_capture_to_bgra.cpp
    src/kfr_capture_to_depth.cpp
    src/kfr_capture_to_timestamp.cpp
    src/kfr_recorder.cpp
    src/kfr_recorder_factory.cpp
    src/kfr_release_capture.cpp
    src/kfr_depth_to_pointcloud.cpp
)

SET(AZURE_KINECT_INTERFACE_INCLUDES
    azure_kinect_interface
    kinect_frame_recipient.h
    kinect_packet.h
    kinect_wrapper.h
    kinect_playback.h
    kfr_capture_to_bgra.h
    kfr_capture_to_depth.h
    kfr_capture_to_timestamp.h
    kfr_recorder.h
    kfr_recorder_factory.h
    kfr_release_capture.h
    kfr_depth_to_pointcloud.h
)

include_directories(
    ${GTK3_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${COMMON_INCLUDES_DIR}
)

add_library(kinectinterface ${AZURE_KINECT_INTERFACE_SRC})
target_link_libraries(kinectinterface ${OpenCV_LIBRARIES})