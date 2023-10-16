cmake_minimum_required(VERSION 2.8)
project(settings_management)

SET(LIB_SETTINGS_MANAGEMENT_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/settings_management
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE
)

SET(SETTINGS_MANAGEMENT_SRC
  src/settings.cpp
)

SET(ROS_INTERFACE_INCLUDES
  settings_management
  settings.h
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(
  ${LIB_SETTINGS_MANAGEMENT_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${COMMON_INCLUDES_DIR}
)

add_library(settings_management ${SETTINGS_MANAGEMENT_SRC})
target_link_libraries(settings_management 
  ${OpenCV_LIBRARIES}
)