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
rosbuild_add_library(${PROJECT_NAME}
	src/lib/FeatureDetector.cpp
  src/lib/FeatureTracker.cpp
	src/lib/Track.cpp
	src/lib/TrackBuilder.cpp
	src/node/FeatureDetectorNode.cpp
	src/node/FeatureTrackerNode.cpp
	src/node/TrackBuilderNode.cpp
  src/node/FeatureViewerNode.cpp
  src/node/TrackViewerNode.cpp
)

rosbuild_add_executable(feature_detector src/node/feature_detector.cpp)
target_link_libraries(feature_detector ${PROJECT_NAME})

rosbuild_add_executable(feature_viewer src/node/feature_viewer.cpp)
target_link_libraries(feature_viewer ${PROJECT_NAME})

rosbuild_add_executable(feature_tracker src/node/feature_tracker.cpp)
target_link_libraries(feature_tracker ${PROJECT_NAME})

rosbuild_add_executable(track_builder src/node/track_builder.cpp)
target_link_libraries(track_builder ${PROJECT_NAME})

rosbuild_add_executable(track_viewer src/node/track_viewer.cpp)
target_link_libraries(track_viewer ${PROJECT_NAME})
