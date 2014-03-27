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
include_directories(/usr/local/cuda/include/)
FIND_LIBRARY(cudalibs cudart /usr/local/cuda/lib64)



rosbuild_add_boost_directories()
rosbuild_add_executable(save_laser src/save_laser_scan.cpp)
rosbuild_link_boost(save_laser system)

rosbuild_add_executable(csm_laser src/csm_laser.cpp)
target_link_libraries(csm_laser ${cudalibs} ${OpenCV_LIBS})
rosbuild_add_openmp_flags(csm_laser)
rosbuild_link_boost(csm_laser system)

rosbuild_add_executable(csmlib src/csm_lib.cpp)
target_link_libraries(csmlib ${cudalibs} ${OpenCV_LIBS})
rosbuild_add_openmp_flags(csmlib)

rosbuild_add_executable(csm_node src/csm_node.cpp src/dbgstream.cpp)
target_link_libraries(csm_node ${cudalibs} ${OpenCV_LIBS} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
rosbuild_add_openmp_flags(csm_node)

set_property(
   TARGET csmlib
   PROPERTY COMPILE_DEFINITIONS PCLNORMAL
   )
set_property(
   TARGET csm_node
   PROPERTY COMPILE_DEFINITIONS PCLNORMAL
   )   
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
