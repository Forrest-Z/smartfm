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

#linuxCNC compatibility
#include_directories(/usr/include/emc2)
include_directories(/home/linuxcnc/linuxcnc/include /home/linuxcnc/linuxcnc/src/hal/components)
LINK_DIRECTORIES(/home/linuxcnc/linuxcnc/lib)
# Uncomment only on the lowlevel computer
#rosbuild_add_executable(halsampler src/halsampler.cpp)
#rosbuild_add_compile_flags(halsampler -DULAPI)
#rosbuild_add_link_flags(halsampler -llinuxcnchal)

# Uncomment only on the lowlevel computer
#rosbuild_add_executable(halstreamer src/halstreamer.cpp)
#rosbuild_add_compile_flags(halstreamer -DULAPI)
#rosbuild_add_link_flags(halstreamer -llinuxcnchal)

#rosbuild_add_executable(halsampler_general src/halsampler_general.cpp)
#rosbuild_add_compile_flags(halsampler_general -DULAPI)
#rosbuild_add_link_flags(halsampler_general -llinuxcnchal)

#rosbuild_add_executable(halstreamer_general src/halstreamer_general.cpp)
#rosbuild_add_compile_flags(halstreamer_general -DULAPI)
#rosbuild_add_link_flags(halstreamer_general -llinuxcnchal)

rosbuild_add_executable(publisher src/publisher.cpp)
rosbuild_add_executable(receiver src/receiver.cpp)