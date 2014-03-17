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

rosbuild_add_executable(keyhook src/keyhook.cpp)
rosbuild_add_executable(keyhook_cmdvel src/keyhook_cmdvel.cpp)
rosbuild_add_executable(joy src/joy.cpp)
rosbuild_add_executable(joy_cmd_vel src/joy_cmd_vel.cpp)
rosbuild_add_executable(joy_pronto src/joy_pronto.cpp)
rosbuild_add_executable(joy_pronto_freedom src/joy_pronto_freedom.cpp)

rosbuild_add_executable(joy_logitech_momo src/joy_logitech_momo.cpp)
rosbuild_add_executable(joy_hal src/joy_hal.cpp)
rosbuild_add_executable(joy_simulator src/joy_simulator.cpp)
rosbuild_add_executable(joy_logitech_momo_iMiev src/joy_logitech_momo_iMiev.cpp)
#rosbuild_add_executable(system_id src/system_id.cpp)
