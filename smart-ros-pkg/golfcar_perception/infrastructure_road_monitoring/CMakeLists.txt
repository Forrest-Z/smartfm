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
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(src)
include_directories(src/lib)

rosbuild_add_library(${PROJECT_NAME}
    src/BackgroundExtraction.cpp
    src/data_def.cpp
    src/BlobFilter.cpp
)

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

ADD_EXECUTABLE(play_video src/nodes/play_video.cpp)
target_link_libraries(play_video ${PROJECT_NAME})

ADD_EXECUTABLE(play_images src/nodes/play_images.cpp)
target_link_libraries(play_images ${PROJECT_NAME})

ADD_EXECUTABLE(background_extractor src/nodes/background_extractor.cpp)
target_link_libraries(background_extractor ${PROJECT_NAME})

ADD_EXECUTABLE(blob_extractor src/nodes/blob_extractor.cpp src/data_msg_conv.cpp)
target_link_libraries(blob_extractor ${PROJECT_NAME})

ADD_EXECUTABLE(blob_area_filter src/nodes/blob_area_filter.cpp src/data_msg_conv.cpp)
target_link_libraries(blob_area_filter ${PROJECT_NAME})

ADD_EXECUTABLE(blob_display src/nodes/blob_display.cpp src/data_msg_conv.cpp)
target_link_libraries(blob_display ${PROJECT_NAME})

ADD_EXECUTABLE(blob_tracker src/nodes/blob_tracker.cpp src/data_msg_conv.cpp src/BlobTracker.cpp)
target_link_libraries(blob_tracker ${PROJECT_NAME})

ADD_EXECUTABLE(track_display src/nodes/track_display.cpp src/data_msg_conv.cpp)
target_link_libraries(track_display ${PROJECT_NAME})

ADD_EXECUTABLE(roi_select src/nodes/roi_select.cpp)
target_link_libraries(roi_select ${PROJECT_NAME})
