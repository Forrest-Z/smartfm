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

#rosbuild_add_library(evg-thin src/evg/evg-thin.cc)
#target_link_libraries(evg-thin ${OpenCV_LIBRARIES})
#rosbuild_add_executable(example src/evg/example.cc)
#target_link_libraries(example evg-thin)

rosbuild_add_library(topo_extractor src/topo_extractor/TopoExtractor.cpp)
target_link_libraries(topo_extractor ${OpenCV_LIBRARIES})
rosbuild_add_executable(road_analyzer src/road_analyzer.cpp)
target_link_libraries(road_analyzer topo_extractor ${OpenCV_LIBRARIES})