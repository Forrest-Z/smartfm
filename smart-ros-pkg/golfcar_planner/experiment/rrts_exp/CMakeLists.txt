cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(ROS_BUILD_TYPE Debug)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/rrts_node_exp.cpp src/dubins_car_exp.cpp src/rrts_exp.hpp src/kdtree.c)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_executable(rrts_node_exp src/main.cpp src/rrts_node_exp.cpp src/dubins_car_exp.cpp src/rrts_exp.hpp src/kdtree.c)
rosbuild_add_executable(simple_goal src/simple_goal.cpp)