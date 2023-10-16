cmake_minimum_required(VERSION 2.8)
project(frame_recipients)

message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

set(LIB_FRAME_RECIPIENTS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/frame_recipients
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

include_directories(${LIB_KINECT_INTERFACE_INCLUDE_DIRS})

#set(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

SET(FRAME_RECIPIENTS_SRC
    src/do_nothing_frame_recipient.cpp
    src/fr_bgra_to_rgb.cpp
    src/fr_display.cpp
    src/fr_rgb_4k_to_1080p.cpp
    src/fr_rgb_4k_to_720p.cpp
    src/fr_splitter.cpp
    src/fr_save_packet.cpp
    src/fr_depth_map_filter.cpp
    src/fr_depth_to_pointcloud.cpp
    src/fr_record_frames.cpp
    src/index_reader_playback.cpp
)

SET(FRAME_RECIPIENTS_INCLUDES
    frame_recipients
    do_nothing_frame_recipient.h
    fr_bgra_to_rgb.h
    fr_display.h
    fr_rgb_4k_to_1080p.h
    fr_rgb_4k_to_720p.h
    fr_splitter.h
    fr_save_packet.h
    fr_depth_map_filter.h
    fr_record_frames.h
    fr_index_reader_playback.h
)

include_directories(
    ${LIB_FRAME_RECIPIENTS_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${COMMON_INCLUDES_DIR}
)

add_library(framerecipients ${FRAME_RECIPIENTS_SRC})
target_link_libraries(framerecipients ${OpenCV_LIBRARIES})