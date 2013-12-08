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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

rosbuild_add_executable(norm_virtual_sensor src/norm_virtual_sensor.cpp)
rosbuild_link_boost(norm_virtual_sensor signals)
#rosbuild_add_executable(carmen src/carmen.cpp src/sensors/AbstractReading.cpp src/sensorstream/CarmenLog.cpp src/sensors/LaserReading.cpp src/geometry/point.cpp )
#rosbuild_link_boost(carmen signals)
#rosbuild_add_executable(carmenToMap src/CarmenToMapNode.cpp src/sensorstream/LogSensorStream.cpp src/sensorstream/CarmenLog.cpp
#                           src/sensors/LaserReading.cpp src/geometry/point.cpp src/sensors/AbstractReading.cpp)
rosbuild_add_executable(odom_baselink src/odom_baselink.cpp)
#rosbuild_add_executable(scan_alignment src/scan_alignment.cpp)
#rosbuild_link_boost(scan_alignment signals)
rosbuild_add_executable(drawAMCLPt src/drawAMCLPt.cpp)
target_link_libraries(drawAMCLPt ${OpenCV_LIBS})
