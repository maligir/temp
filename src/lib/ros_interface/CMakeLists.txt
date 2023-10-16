cmake_minimum_required(VERSION 2.8)
project(ros_interface)

SET(LIB_ROS_INTERFACE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/ros_interface
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE
)

SET(ROS_INTERFACE_SRC
  src/rfr_get_frame_info.cpp
  src/rfr_sync_image_frames.cpp
  src/rosbag_playback.cpp
)

SET(ROS_INTERFACE_INCLUDES
  ros_interface
  rfr_get_frame.h
  rfr_sync_image_frames.h
  ros_frame_recipient.h
  ros_packet.h
  rosbag_playback.h
)

SET(ROS_INCLUDES_DIR /opt/ros/melodic/include)
SET(ROS_LIBRARIES_DIR /opt/ros/melodic/lib)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(rosbag REQUIRED)
# find_package(cv_bridge REQUIRED)

include_directories(
  ${LIB_ROS_INTERFACE_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${COMMON_INCLUDES_DIR}
  ${ROS_INCLUDES_DIR}
  ${ROS_LIBRARIES_DIR}
  ${rosbag_INCLUDE_DIRS}
  # ${cv_bridge_INCLUDE_DIRS}
)

add_library(rosinterface ${ROS_INTERFACE_SRC})
target_link_libraries(rosinterface 
  ${OpenCV_LIBRARIES}
  ${rosbag_LIBRARIES}
  -I${ROS_INCLUDES_DIR}
  -L${ROS_LIBRARIES_DIR}
  -lroscpp
  -lrostime
  -lrosbag
  -lboost_system
  -lcv_bridge
)