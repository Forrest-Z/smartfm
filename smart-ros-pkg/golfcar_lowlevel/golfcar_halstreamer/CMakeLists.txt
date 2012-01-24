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

#linuxCNC compatibility
include_directories(/usr/include/emc2)

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
#rosbuild_add_executable(golfcar_halstreamer src/golfcar_halstreamer.cpp)
rosbuild_add_executable(keyhook src/keyhook.cpp)
rosbuild_add_executable(keyhook_cmdvel src/keyhook_cmdvel.cpp)
rosbuild_add_executable(joy src/joy.cpp)
rosbuild_add_executable(joy_cmd_vel src/joy_cmd_vel.cpp)
rosbuild_add_executable(joy_multichannel src/joy_multichannel.cpp)


# Uncomment the following lines on the lowlevel computer only
#rosbuild_add_executable(golfcar_halstreamer_3channel src/golfcar_halstreamer_3channel.cpp)
#rosbuild_add_compile_flags(golfcar_halstreamer_3channel -DULAPI)
#rosbuild_add_link_flags(golfcar_halstreamer_3channel -lemchal)
