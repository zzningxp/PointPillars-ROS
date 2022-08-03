cmake_minimum_required(VERSION 2.8.3)
project(test_point_pillars)

find_package(CUDA REQUIRED)
find_library(CUDNN_LIBRARY
  NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
  PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
  PATH_SUFFIXES lib lib64 bin
  DOC "CUDNN library."
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  lidar_point_pillars
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    roslib
    lidar_point_pillars
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

set(SOURCE_FILES
  nodes/test_point_pillars_node.cpp
)

add_executable(test_point_pillars
  ${SOURCE_FILES}
)

target_link_libraries(test_point_pillars
  point_pillars_lib
  ${catkin_LIBRARIES}
  ${NVINFER}
  ${NVONNXPARSERS}
  ${CUDA_LIBRARIES}
  # ${CUDA_CUBLAS_LIBRARIES}
  ${CUDA_curand_LIBRARY}
  ${CUDNN_LIBRARY}
)
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)