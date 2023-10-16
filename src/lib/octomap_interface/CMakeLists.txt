cmake_minimum_required(VERSION 2.8)
project(octomap_interface)

set(LIB_OCTOMAP_INTERFACE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE
)

include_directories(${LIB_OCTOMAP_INTERFACE_INCLUDE_DIRS})

find_package(OpenCV 4 REQUIRED)
find_package(octomap REQUIRED)

SET(OCTOMAP_INTERFACE_INCLUDES
    octmap_utils.h
)

# add_library(octomapinterface)
