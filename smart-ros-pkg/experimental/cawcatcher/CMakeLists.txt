cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(listener src/listener.cpp)
rosbuild_add_executable(joy_command src/joy_command.cpp)
rosbuild_add_executable(Sweep src/Sweep.cpp)
rosbuild_add_executable(Sweep2 src/Sweep2.cpp)
#rosbuild_add_executable(adaptive src/adaptive.cpp)
rosbuild_add_executable(adaptive_modified src/adaptive_modified.cpp)

rosbuild_add_executable(publishFakeOdom src/publishFakeOdom.cpp)
rosbuild_add_executable(listenerFake src/listenerFake.cpp)

rosbuild_add_executable(steady src/steady.cpp)
rosbuild_add_executable(pointcloud2converter src/pointcloud2converter.cpp)

#rosbuild_add_executable(pcl_accumulator src/pcl_accumulator.cpp)
rosbuild_add_executable(Xiaotong src/Xiaotong.cpp)
