cmake_minimum_required(VERSION 2.8)
project(mapping)

set(LIB_MAPPING_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE
)

include_directories(${LIB_MAPPING_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Pangolin REQUIRED)
# find_package(Ceres REQUIRED)

find_library(FRAME_RECIPIENTS_LIB framerecipients)
find_library(ORBSLAM_LIB ORB_SLAM3)

SET(MAPPING_SRC
    src/occupation_map_2d.cpp
    # src/kfr_track_icp_slam.cpp
    src/fr_map_merging_handler.cpp
    src/mm_point_cloud_iou.cpp
)

message("NANOFLANN_INCLUDE_DIRS: ${NANOFLANN_INCLUDE_DIRS}")

SET(MAPPING_INCLUDES
    occupation_map_2d.h
    # ceres_utils.h
    # kfr_track_icp_slam.h
    map_merger.h
    fr_map_merging_handler.h
    mm_point_cloud_iou.h
    nanoflann.hpp
)

message("EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
message("CERES_INCLUDE_DIRS: ${CERES_INCLUDE_DIRS}")
message("CERES_LIBRARIES: ${CERES_LIBRARIES}")

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${LIB_FRAME_RECIPIENTS_INCLUDE_DIRS}
    ${LIB_ORBSLAM_INTERFACE_INCLUDE_DIRS}
    ${ORB_SLAM3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    # ${CERES_INCLUDE_DIRS}
    ${COMMON_INCLUDES_DIR}
)

add_library(mapping ${MAPPING_SRC})
target_link_libraries(mapping
    ${Pangolin_LIBRARIES}
    # ${CERES_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
