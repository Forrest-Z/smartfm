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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(/usr/local/include/gstreamer-1.0
 /usr/include/glib-2.0 /usr/lib/x86_64-linux-gnu/glib-2.0/include)
#  
# rosbuild_add_executable(gscam src/gscam.cpp)
# target_link_libraries(gscam gstreamer-1.0  gstbase-1.0 gstapp-1.0)

rosbuild_add_executable(gst_opencv_streamer src/gst_opencv_streamer.cpp)
target_link_libraries(gst_opencv_streamer gstreamer-1.0  gstbase-1.0 gstapp-1.0 gstvideo-1.0 gstrtsp-1.0 gstrtspserver-1.0)

rosbuild_add_executable(gst_opencv_receiver src/gst_opencv_receiver.cpp)
target_link_libraries(gst_opencv_receiver gstreamer-1.0  gstbase-1.0 gstapp-1.0 gstvideo-1.0 gstrtsp-1.0 gstrtspserver-1.0)
