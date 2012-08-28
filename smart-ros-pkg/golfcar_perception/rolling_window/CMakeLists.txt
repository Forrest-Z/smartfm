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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_add_executable(road_surface src/road_surface.cpp)
rosbuild_link_boost(road_surface signals)
rosbuild_add_executable(rolling_window_pcl src/rolling_window_pcl.cpp)
rosbuild_link_boost(rolling_window_pcl signals)

rosbuild_add_executable(single_window src/single_window.cpp)
rosbuild_link_boost(single_window signals)
rosbuild_add_executable(quick_process src/quick_process.cpp)
rosbuild_link_boost(quick_process signals)
#rosbuild_add_executable(ldf src/laser_dynamic_object_filter.cpp)
#target_link_libraries (le pcl_visualization gomp)
#rosbuild_add_compile_flags(le -lpcl_visualization)
#target_link_libraries(example ${PROJECT_NAME})
