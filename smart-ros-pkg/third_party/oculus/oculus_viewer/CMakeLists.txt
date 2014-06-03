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

rosbuild_add_boost_directories()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(image_distort_viewer src/distort_image.cpp
src/viewer.cpp src/image_distort_viewer.cpp)
rosbuild_add_executable(stereo_sync src/stereo_sync.cpp)
rosbuild_link_boost(stereo_sync signals)

include_directories(/usr/include/gstreamer-0.10 /usr/include/glib-2.0 
/usr/lib/x86_64-linux-gnu/glib-2.0/include/ /usr/include/libxml2)
rosbuild_add_executable(rtsp_gstreamer src/rtsp_gstreamer.cpp)
target_link_libraries(rtsp_gstreamer gstrtspserver-0.10)

rosbuild_add_executable(rtsp_grabber src/rtsp_grabber.cpp)
#target_link_libraries(example ${PROJECT_NAME})
# 
# CPPFLAGS += -I. -I$(top_builddir)/include -I/usr/include/gstreamer-0.10 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/libxml2 -I/usr/lib/glib-2.0/include -I/usr/include/gstreamer-0.10/gst/rtsp-server/
# 
# OBJ = rtsptest.o
# 
# %.o: %.c $(DEPS)
# 	$(CC) -c -o $@ $< $(CPPFLAGS)
# 
# rtsptes	t: $(OBJ)
# 	$(CC) -o $@ $^ $(LDFLAGS) $(CPPFLAGS) -pthread -lstdc++ -lgstreamer-0.10 -lgstrtsp-0.10 -lgstrtspserver-0.10 -lgobject-2.0 -lgmodule-2.0 -lgthread-2.0 -lrt -lxml2 -lglib-2.0 -lm
# 
# clean:
# 	rm *.o
# 	rm test

rosbuild_add_executable(network_delay_sender src/network_delay_sender.cpp)
rosbuild_add_executable(network_delay_receiver src/network_delay_receiver.cpp)