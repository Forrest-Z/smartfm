cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
include_directories(/usr/include/vtk-5.8)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
CUDA_ADD_LIBRARY(raster_map src/raster_pc_grayscale_v2.cu)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(recovering_map src/recovering_map.cpp src/dbgstream.cpp)
target_link_libraries(recovering_map ${OpenCV_LIBS} raster_map)
rosbuild_add_executable(saving_map src/saving_map.cpp)

#rosbuild_add_executable(test_mrpt_icp src/test_mrpt_icp.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(lidar_rolling_window src/lidar_rolling_window.cpp)
rosbuild_link_boost(lidar_rolling_window signals)

CUDA_ADD_LIBRARY(csm_lib src/csm_cuda/csm_lib.cu)
rosbuild_add_executable(map_merging_optimize src/map_merging_optimize.cpp)
rosbuild_link_boost(map_merging_optimize signals)
target_link_libraries(map_merging_optimize csm_lib)

rosbuild_add_executable(merging_quality_evaluation src/merging_quality_evaluation.cpp)
