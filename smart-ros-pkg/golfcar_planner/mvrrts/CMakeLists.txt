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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules(BOT2 REQUIRED bot2-lcmgl-client bot2-core bot2-vis bot2-lcmgl-renderer)
#include_directories(${BOT2_INCLUDE_DIRS})
#link_directories(${BOT2_LIBRARY_DIRS})
#target_link_libraries(mvrrts ${BOT2_LIBRARIES})

rosbuild_add_executable(mvrrts_node src/mvrrts_node.cpp src/mvrrts.cpp src/mvdubins.cpp src/automata.cpp)
