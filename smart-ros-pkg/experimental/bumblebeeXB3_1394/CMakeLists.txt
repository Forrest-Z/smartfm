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
set(PGR_LIB src/pgr_conversions.cpp src/pgr_registers.cpp src/pgr_stereocam.cpp)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(xb3 src/xb3.cpp ${PGR_LIB})
#rosbuild_add_link_flags(xb3 -ldc1394)
target_link_libraries(xb3 pnmutils ${OpenCV_LIBS} triclops dc1394)
rosbuild_add_executable(time_server src/time_server.cpp)
rosbuild_add_executable(triclopsRect src/triclopsRect.cpp ${PGR_LIB})
target_link_libraries(triclopsRect pnmutils ${OpenCV_LIBS} triclops dc1394)
