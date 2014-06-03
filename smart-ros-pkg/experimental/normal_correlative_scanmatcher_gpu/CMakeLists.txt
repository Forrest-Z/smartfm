cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(ISAM_LIBS isam cholmod colamd amd lapack blas rt)
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(/usr/include/vtk-5.8 /usr/local/cuda/include/)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
FIND_LIBRARY(cudalibs cudart /usr/local/cuda/lib64)
# CUDA_ADD_LIBRARY(raster_map src/raster_pc_grayscale_v2.cu)
INCLUDE_DIRECTORIES( "/usr/include/mysql" )
rosbuild_add_executable(isam_full src/isam_full.cpp src/dbgstream.cpp)
target_link_libraries(isam_full ${ISAM_LIBS} ${cudalibs} ${OpenCV_LIBS} mysqlpp)

set_property(
   TARGET isam_full
   PROPERTY COMPILE_DEFINITIONS PCLNORMAL
   )   
rosbuild_add_openmp_flags(isam_full)
# 
# rosbuild_add_executable(all_matching src/all_matching.cpp src/dbgstream.cpp)
# target_link_libraries(all_matching ${ISAM_LIBS} ${OpenCV_LIBS} mysqlpp raster_map)

#rosbuild_add_executable(simple_occupancy_grid_construction src/simple_occupancy_grid_construction.cpp)
