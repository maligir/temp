cmake_minimum_required(VERSION 2.8)
project(app)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# set(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
# find_package(Ceres REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Pangolin REQUIRED)
find_package(octomap REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

find_library(ORBSLAM_LIB ORB_SLAM3)
find_library(tesseract_lib libtesseract)
find_library(vision_geometry vision_geometry)

message("LIB_KINECT_INTERFACE_INCLUDE_DIRS: ${LIB_KINECT_INTERFACE_INCLUDE_DIRS}")
message("LIB_ORBSLAM_INTERFACE_INCLUDE_DIRS: ${LIB_ORBSLAM_INTERFACE_INCLUDE_DIRS}")
message("LIB_OCTOMAP_INTERFACE_INCLUDE_DIRS: ${LIB_OCTOMAP_INTERFACE_INCLUDE_DIRS}")
message("LIB_MAPPING_INCLUDE_DIRS: ${LIB_MAPPING_INCLUDE_DIRS}")
message("LIB_SEMANTIC_PROCESSING_INCLUDE_DIRS: ${LIB_SEMANTIC_PROCESSING_INCLUDE_DIRS}")
# message("CERES_INCLUDE_DIRS: ${CERES_INCLUDE_DIRS}")
# message("CERES_LIBRARIES: ${CERES_LIBRARIES}")

message("Tesseract_INCLUDE_DIRS: ${Tesseract_INCLUDE_DIRS}")

include_directories(
    ${LIB_FRAME_RECIPIENTS_INCLUDE_DIRS}
    ${LIB_ROS_INTERFACE_INCLUDE_DIRS}
    ${LIB_KINECT_INTERFACE_INCLUDE_DIRS}
    ${LIB_ORBSLAM_INTERFACE_INCLUDE_DIRS}
    ${LIB_OCTOMAP_INTERFACE_INCLUDE_DIRS}
    ${LIB_APRILTAG_INTERFACE_INCLUDE_DIRS}
    ${LIB_SETTINGS_MANAGEMENT_INCLUDE_DIRS}
    ${LIB_MAPPING_INCLUDE_DIRS}
    ${LIB_SEMANTIC_PROCESSING_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${VISION_GEOMETRY_INCLUDE_DIRS}
    ${APRIL_TAG_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}/eigen3
    ${ORB_SLAM3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${COMMON_INCLUDES_DIR}
    ${Tesseract_INCLUDE_DIRS}
)

add_executable(write_video write_video.cpp)
target_link_libraries(write_video
    framerecipients
    rosinterface
    settings_management
    kinectinterface
    k4a
    k4arecord
    ${OpenCV_LIBRARIES}
    ${GTK3_LIBRARIES}
    pthread
)

add_executable(playback play_video.cpp)
target_link_libraries(playback
    settings_management
    rosinterface
    framerecipients
    kinectinterface
    orbslaminterface
    mapping
    k4a
    k4arecord
    pthread
    ORB_SLAM3
    yolo_lib
    libtesseract
    semantic_processing
    lept
    curl
    ${OCTOMAP_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${CERES_LIBRARIES}
)

add_executable(map_k4a map_k4a.cpp)
target_link_libraries(map_k4a
    settings_management
    framerecipients
    kinectinterface
    orbslaminterface
    mapping
    k4a
    k4arecord
    pthread
    ORB_SLAM3
    yolo_lib
    libtesseract
    semantic_processing
    lept
    curl
    ${OCTOMAP_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${CERES_LIBRARIES}
)

add_executable(map_ros map_ros.cpp)
target_link_libraries(map_ros
    settings_management
    rosinterface
    framerecipients
    orbslaminterface
    mapping
    pthread
    ORB_SLAM3
    yolo_lib
    libtesseract
    semantic_processing
    lept
    curl
    ${OCTOMAP_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${CERES_LIBRARIES}
)

add_executable(get_semantics get_semantics.cpp)
target_link_libraries(get_semantics
    framerecipients
    rosinterface
    settings_management
    kinectinterface
    orbslaminterface
    mapping
    k4a
    k4arecord
    pthread
    ORB_SLAM3
    yolo_lib
    libtesseract
    semantic_processing
    lept
    curl
    ${OCTOMAP_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${CERES_LIBRARIES}
)

add_executable(label_placards label_placards.cpp)
target_link_libraries(label_placards
    framerecipients
    rosinterface
    settings_management
    kinectinterface
    k4a
    k4arecord
    pthread
    ${OpenCV_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

add_executable(test_live test_live.cpp)
target_link_libraries(test_live
    framerecipients
    rosinterface
    settings_management
    kinectinterface
    orbslaminterface
    k4a
    k4arecord
    ${OpenCV_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${GTK3_LIBRARIES}
    ORB_SLAM3
)

add_executable(test_orbslam test_orbslam.cpp)
target_link_libraries(test_orbslam
    framerecipients
    rosinterface
    settings_management
    kinectinterface
    orbslaminterface
    k4a
    k4arecord
    mapping
    pthread
    ORB_SLAM3
    ${OpenCV_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${GTK3_LIBRARIES}
)

add_executable(test_map_merging test_map_merging.cpp)
target_link_libraries(test_map_merging
    framerecipients
    rosinterface
    settings_management
    kinectinterface
    orbslaminterface
    mapping
    k4a
    k4arecord
    pthread
    ORB_SLAM3
    ${OpenCV_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${GTK3_LIBRARIES}
)

add_executable(test_placard_extraction test_placard_extraction.cpp)
target_link_libraries(test_placard_extraction
    framerecipients
    rosinterface
    settings_management
    kinectinterface
    k4a
    k4arecord
    ${OpenCV_LIBRARIES}
    yolo_lib
    libtesseract
    semantic_processing
    lept
    curl
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

add_executable(test_yolov5 test_yolov5.cpp)
target_link_libraries(test_yolov5
    framerecipients
    rosinterface
    settings_management
    semantic_processing
    ${OpenCV_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

add_executable(view_images view_images_at_time.cpp)
target_link_libraries(view_images
    framerecipients
    rosinterface
    settings_management
    kinectinterface
    k4a
    k4arecord
    semantic_processing
    ${OpenCV_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

add_executable(get_frames get_frames.cpp)
target_link_libraries(get_frames
    framerecipients
    rosinterface
    settings_management
    kinectinterface
    k4a
    k4arecord
    ${OpenCV_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    vision_geometry
    apriltag
    apriltaginterface
)

# add_executable(test_icp test_icp.cpp)
# target_link_libraries(test_icp
#     kinectinterface
#     orbslaminterface
#     mapping
#     k4a
#     k4arecord
#     pthread
#     ORB_SLAM3
#     ${OCTOMAP_LIBRARIES}
#     ${OpenCV_LIBRARIES}
#     ${Pangolin_LIBRARIES}
#     ${GTK3_LIBRARIES}
#     ${Boost_FILESYSTEM_LIBRARY}
#     ${Boost_SYSTEM_LIBRARY}
#     ${CERES_LIBRARIES}
# )