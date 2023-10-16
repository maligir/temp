cmake_minimum_required(VERSION 2.8)
set(CMAKE_C_COMPILER "/usr/bin/gcc-6")
project(placard_discovery_top)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++17 -m64")

message("CMAKE_CXX_STANDARD: ${CMAKE_CXX_STANDARD}")
message("Vision Geometry")
add_subdirectory(third_party/vision_geometry)
message("Pangolin")
add_subdirectory(third_party/Pangolin)
message("DBoW2")
add_subdirectory(third_party/ORB_SLAM3/Thirdparty/DBoW2)
message("g2o")
add_subdirectory(third_party/ORB_SLAM3/Thirdparty/g2o)
message("Sophus")
add_subdirectory(third_party/ORB_SLAM3/Thirdparty/Sophus)
message("ORB_SLAM3")
add_subdirectory(third_party/ORB_SLAM3/)
message("octomap")
add_subdirectory(third_party/octomap/)
# message("caffe")
# add_subdirectory(third_party/caffe/)
message("tesseract")
add_subdirectory(third_party/tesseract/)
message("apriltag")
add_subdirectory(third_party/apriltag/)

set(ORB_SLAM3_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/third_party/ORB_SLAM3/
    ${CMAKE_SOURCE_DIR}/third_party/ORB_SLAM3/include/
    ${CMAKE_SOURCE_DIR}/third_party/ORB_SLAM3/include/CameraModels/
    ${CMAKE_SOURCE_DIR}/third_party/ORB_SLAM3/Thirdparty/Sophus
)

set(VISION_GEOMETRY_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/vision_geometry/include)

set(APRIL_TAG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/apriltag)

set(OCTOMAP_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/third_party/octomap/octomap/include/
)

set(NANOFLANN_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/third_party/nanoflann/include/
)

set(Tesseract_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/third_party/tesseract/include/
)

add_subdirectory(src)

make_directory(${CMAKE_CURRENT_BINARY_DIR}/share)
make_directory(${CMAKE_CURRENT_BINARY_DIR}/share/lib)

exec_program("tar xf ${CMAKE_SOURCE_DIR}/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz --directory ${CMAKE_CURRENT_BINARY_DIR}/share/lib")
