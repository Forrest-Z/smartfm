cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(CMAKE_CXX_FLAGS_DEBUG "-std=c++0x -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++0x -O3")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(_${PROJECT_NAME} src/Paths_and_Regions.cpp)
rosbuild_add_library(_${PROJECT_NAME} src/Maps.cpp)
rosbuild_add_library(_${PROJECT_NAME} src/DubinsCar.cpp)
rosbuild_add_library(_${PROJECT_NAME} src/DubinsCar_MVRRTs.cpp)

rosbuild_add_executable(mvrrts_v2 src/MVRRTs_v2.cpp)

target_link_libraries(mvrrts_v2 _${PROJECT_NAME})

