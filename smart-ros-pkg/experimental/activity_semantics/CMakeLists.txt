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

find_package(OpenCV REQUIRED)


rosbuild_add_library(local_track_show src/tools/local_track_show.cpp)
rosbuild_add_library(global_track_show src/tools/global_track_show.cpp)

rosbuild_add_executable(simu_track_collector src/tools/simul_track_collector.cpp)

rosbuild_add_executable(ped_collector src/activity_collector/pedestrian_collector.cpp)
target_link_libraries(ped_collector ${OpenCV_LIBRARIES} local_track_show global_track_show)

#rosbuild_add_library(AM_learner src/semantic_learner/AM_learner.cpp)
rosbuild_add_library(AM_learner src/semantic_learner/AM_learner_NBCs.cpp)
rosbuild_add_library(track_processor src/semantic_learner/track_processor.cpp)
rosbuild_add_executable(ped_semantics src/semantic_learner/ped_semantics.cpp)
target_link_libraries(ped_semantics ${OpenCV_LIBRARIES} local_track_show global_track_show AM_learner track_processor)

