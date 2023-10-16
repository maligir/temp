cmake_minimum_required(VERSION 2.8.12)
project(semantic_processing)

set(LIB_SEMANTIC_PROCESSING_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE
)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)

include_directories(${LIB_SEMANTIC_PROCESSING_INCLUDE_DIRS})

find_package(OpenCV 4 REQUIRED)
find_package(CUDA QUIET)
if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
  )
  # add_definitions(-DGPU)
else()
#   list(APPEND LIBRARIES "m")
endif()
find_package ( X11 REQUIRED )
# find_package(Caffe REQUIRED)
# add_definitions(-DOPENCV)
# add_definitions(-O4 -g)

find_library(tesseract_lib libtesseract)
find_library(FRAME_RECIPIENTS_LIB framerecipients)

find_path(DARKNET_PATH
  NAMES "README.md"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/darknet")

include_directories(
    include
    ${COMMON_INCLUDES_DIR}
    ${OpenCV_INCLUDE_DIR}
    ${X11_INCLUDE_DIR}
    ${LIB_FRAME_RECIPIENTS_INCLUDE_DIRS}
    # ${CAFFE_INCLUDE_DIRS}
    ${Tesseract_INCLUDE_DIRS}
)

SET(SEMANTIC_PROCESSING_SRC
  src/yolo_observer.cpp
  src/yolov5_observer.cpp
  src/fr_get_bounding_boxes.cpp
  # src/image_segmentation.cpp
  src/placard_database.cpp
  src/fr_extract_placard.cpp
)

SET(SEMANTIC_PROCESSING_INCLUDES
  observer.h
  yolo_observer.h
  yolov5_observer.h
  fr_get_bounding_boxes.h
  image_segmentation.h
  placard_database.h
  fr_extract_placard.h
)

set(DARKNET_CORE_FILES
    ${DARKNET_PATH}/src/activation_layer.c        ${DARKNET_PATH}/src/im2col.c
    ${DARKNET_PATH}/src/activations.c             ${DARKNET_PATH}/src/image.c
    ${DARKNET_PATH}/src/avgpool_layer.c           ${DARKNET_PATH}/src/layer.c
    ${DARKNET_PATH}/src/batchnorm_layer.c         ${DARKNET_PATH}/src/list.c
    ${DARKNET_PATH}/src/blas.c                    ${DARKNET_PATH}/src/local_layer.c
    ${DARKNET_PATH}/src/box.c                     ${DARKNET_PATH}/src/lstm_layer.c
    ${DARKNET_PATH}/src/col2im.c                  ${DARKNET_PATH}/src/matrix.c
    ${DARKNET_PATH}/src/connected_layer.c         ${DARKNET_PATH}/src/maxpool_layer.c
    ${DARKNET_PATH}/src/convolutional_layer.c     ${DARKNET_PATH}/src/network.c
    ${DARKNET_PATH}/src/cost_layer.c              ${DARKNET_PATH}/src/normalization_layer.c
    ${DARKNET_PATH}/src/crnn_layer.c              ${DARKNET_PATH}/src/option_list.c
    ${DARKNET_PATH}/src/crop_layer.c              ${DARKNET_PATH}/src/parser.c
    ${DARKNET_PATH}/src/cuda.c                    ${DARKNET_PATH}/src/region_layer.c
    ${DARKNET_PATH}/src/data.c                    ${DARKNET_PATH}/src/reorg_layer.c
    ${DARKNET_PATH}/src/deconvolutional_layer.c   ${DARKNET_PATH}/src/rnn_layer.c
    ${DARKNET_PATH}/src/demo.c                    ${DARKNET_PATH}/src/route_layer.c
    ${DARKNET_PATH}/src/detection_layer.c         ${DARKNET_PATH}/src/shortcut_layer.c
    ${DARKNET_PATH}/src/dropout_layer.c           ${DARKNET_PATH}/src/softmax_layer.c
    ${DARKNET_PATH}/src/gemm.c                    ${DARKNET_PATH}/src/tree.c
    ${DARKNET_PATH}/src/gru_layer.c               ${DARKNET_PATH}/src/utils.c
    ${DARKNET_PATH}/src/upsample_layer.c          ${DARKNET_PATH}/src/logistic_layer.c
    ${DARKNET_PATH}/src/l2norm_layer.c            ${DARKNET_PATH}/src/yolo_layer.c
    ${DARKNET_PATH}/src/iseg_layer.c              ${DARKNET_PATH}/src/image_opencv.cpp

    ${DARKNET_PATH}/examples/art.c                ${DARKNET_PATH}/examples/lsd.c
    ${DARKNET_PATH}/examples/nightmare.c          ${DARKNET_PATH}/examples/instance-segmenter.c
    ${DARKNET_PATH}/examples/captcha.c            ${DARKNET_PATH}/examples/regressor.c
    ${DARKNET_PATH}/examples/cifar.c              ${DARKNET_PATH}/examples/rnn.c
    ${DARKNET_PATH}/examples/classifier.c         ${DARKNET_PATH}/examples/segmenter.c
    ${DARKNET_PATH}/examples/coco.c               ${DARKNET_PATH}/examples/super.c
    ${DARKNET_PATH}/examples/darknet.c            ${DARKNET_PATH}/examples/tag.c
    ${DARKNET_PATH}/examples/detector.c           ${DARKNET_PATH}/examples/yolo.c
    ${DARKNET_PATH}/examples/go.c
)


set(DARKNET_CUDA_FILES
    ${DARKNET_PATH}/src/activation_kernels.cu     ${DARKNET_PATH}/src/crop_layer_kernels.cu
    ${DARKNET_PATH}/src/avgpool_layer_kernels.cu  ${DARKNET_PATH}/src/deconvolutional_kernels.cu
    ${DARKNET_PATH}/src/blas_kernels.cu           ${DARKNET_PATH}/src/dropout_layer_kernels.cu
    ${DARKNET_PATH}/src/col2im_kernels.cu         ${DARKNET_PATH}/src/im2col_kernels.cu
    ${DARKNET_PATH}/src/convolutional_kernels.cu  ${DARKNET_PATH}/src/maxpool_layer_kernels.cu
)

if (CUDA_FOUND)

  message("CUDA_TOOLKIT_ROOT_DIR: ${CUDA_TOOLKIT_ROOT_DIR}")
  link_directories(
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
  )

  cuda_add_library(yolo_lib
    ${DARKNET_CORE_FILES}
    ${DARKNET_CUDA_FILES}
  )

  add_library(cudart SHARED IMPORTED)
  set_property(TARGET cudart PROPERTY IMPORTED_LOCATION "${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcudart.so")
  add_library(cublas SHARED IMPORTED)
  set_property(TARGET cublas PROPERTY IMPORTED_LOCATION "${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcublas.so")
  add_library(curand SHARED IMPORTED)
  set_property(TARGET curand PROPERTY IMPORTED_LOCATION "${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcurand.so")

  target_link_libraries(yolo_lib
    cuda
    cudart
    cublas
    curand
    ${X11_LIBRARIES}
    ${OpenCV_LIBS}
  )

  target_compile_definitions(yolo_lib PRIVATE
    -DGPU
    -DOPENCV
  )

else()

  add_library(yolo_lib
    ${DARKNET_CORE_FILES}
  )

  target_link_libraries(yolo_lib
    ${X11_LIBRARIES}
    ${OpenCV_LIBS}
  )

  target_compile_definitions(yolo_lib PRIVATE
    -DOPENCV
  )

endif()

target_compile_definitions(yolo_lib PUBLIC
  DARKNET_FILE_PATH="${DARKNET_PATH}"
)

target_include_directories(yolo_lib PUBLIC
  ${DARKNET_PATH}/src
  ${DARKNET_PATH}/include
)

if (CUDA_FOUND)

cuda_add_library(${PROJECT_NAME} STATIC
  ${SEMANTIC_PROCESSING_SRC}
)

else()

add_library(${PROJECT_NAME} STATIC
  ${SEMANTIC_PROCESSING_SRC}
)

endif()

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS} 
  ${VISION_GEOMETRY_LIBRARY}
  ${X11_LIBRARIES}
  yolo_lib
  # ${Caffe_LIBRARIES}
  libtesseract
  lept
  curl
)