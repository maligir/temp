cmake_minimum_required(VERSION 2.8.3)
project(vision_geometry)

add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)  # This should work on both 3 and 4

message("Eigen3" ${EIGEN3_INCLUDE_DIR})


include_directories(
    include
    ${Ceres_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    #/usr/include/eigen3
    #${catkin_INCLUDE_DIRS}
)

SET(MAPPING_SOURCES
    #src/CalibrationModel.cc
    src/CameraIntrinsics.cc
    src/CameraOptimizationParameters.cc
    src/CVUtil.cc
    src/DistortionModel.cc
    src/HomCartShortcuts.cc
    src/HomographyShortcuts.cc
    src/LinearAlgebraShortcuts.cc
    src/ModelChessboard.cc
    src/OptRTDistCostFunctor.cc
    src/RigidTrans.cc
    src/SingleCameraOptimizer.cc
    src/TransformShortcuts.cc
    src/Util.cc
    src/ZhangLinearEstimate.cc
    src/ModelPlaque.cc)

add_library(vision_geometry ${MAPPING_SOURCES})
add_executable(mapping_calibrator ${MAPPING_SOURCES} src/mapping_calibration.cc)


target_link_libraries(vision_geometry
    ceres
    ${Ceres_LIBS}
    ${OpenCV_LIBS}
    ${Eigen3_LIBS}
    )

target_link_libraries(mapping_calibrator
    ceres
    ${Ceres_LIBS}
    ${OpenCV_LIBS}
    )

install(TARGETS vision_geometry DESTINATION lib)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/vision_geometry
  FILES_MATCHING PATTERN "*.h")

#install(TARGETS
#        mapping_calibrator
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
