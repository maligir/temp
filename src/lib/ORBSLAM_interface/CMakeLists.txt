cmake_minimum_required(VERSION 2.8)
project(orbslam_interface)

set(LIB_ORBSLAM_INTERFACE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE
)

include_directories(${LIB_ORBSLAM_INTERFACE_INCLUDE_DIRS})

#set(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
find_package(Pangolin REQUIRED)

find_library(FRAME_RECIPIENTS_LIB framerecipients)
find_library(ORBSLAM_LIB ORB_SLAM3)

SET(ORBSLAM_INTERFACE_SRC
    src/fr_track_orb_slam.cpp
)

SET(ORBSLAM_INTERFACE_INCLUDES
    track_orb_slam.h
    orbslam_utils.h
)

include_directories(
    ${GTK3_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LIB_FRAME_RECIPIENTS_INCLUDE_DIRS}
    ${ORB_SLAM3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${COMMON_INCLUDES_DIR}
)

add_library(orbslaminterface ${ORBSLAM_INTERFACE_SRC})
target_link_libraries(orbslaminterface
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)