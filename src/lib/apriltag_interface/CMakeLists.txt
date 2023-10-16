cmake_minimum_required(VERSION 2.8)
project(apriltag_interface)

message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

set(LIB_APRILTAG_INTERFACE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/apriltag_interface
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

include_directories(${LIB_APRILTAG_INTERFACE_INCLUDE_DIRS})

#set(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)

# find_library(vision_geometry vision_geometry)

pkg_search_module(apriltag REQUIRED apriltag)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

SET(APRILTAG_INTERFACE_SRC
    src/fr_apriltag_pose.cpp
    src/fr_april_tag.cpp
    src/april_tag_manager.cpp
)

SET(APRILTAG_INTERFACE_INCLUDES
    apriltag_interface
    fr_apriltag_pose.h
    fr_april_tag.h
    april_tag_manager.h
)

include_directories(
    ${GTK3_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${VISION_GEOMETRY_INCLUDE_DIRS}
    ${APRIL_TAG_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${COMMON_INCLUDES_DIR}
    ${CMAKE_SOURCE_DIR}/third_party/ORB_SLAM3/Thirdparty/Sophus
)

add_library(apriltaginterface ${APRILTAG_INTERFACE_SRC})
target_link_libraries(apriltaginterface ${OpenCV_LIBRARIES} apriltag vision_geometry)