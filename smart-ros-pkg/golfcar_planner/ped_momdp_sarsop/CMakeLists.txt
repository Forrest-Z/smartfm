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
#set(CMAKE_C_COMPILER "gcc-4.8")
#set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_C_COMPILER "colorgcc")
set(CMAKE_CXX_COMPILER "colorgcc")
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# search header files in those sub directories
# Maybe a glob could be possible
INCLUDE_DIRECTORIES(
	src/despot/
    src/planner/
)

FILE(GLOB LIBDESPOT_SRCS src/despot/*.cpp src/despot/util/*.cpp src/planner/*.cpp)

# There are too many compiler warnings with problem-solver code, so let's disable some:
#set(WARNING_FLAGS "-Wno-sign-compare -Wno-unused-variable -Wno-ignored-qualifiers -Wno-write-strings")

add_definitions(-DDEBUG_LOG_ON)

#set(PED_COMPILE_FLAGS -std=c++0x -O0 -ggdb  -fbounds-check)
set(PED_COMPILE_FLAGS -std=c++0x -O3)


#rosbuild_add_library(pomcp ${LIBPOMCP_SRCS})
rosbuild_add_library(despot ${LIBDESPOT_SRCS})
rosbuild_add_compile_flags(despot ${PED_COMPILE_FLAGS})
rosbuild_add_executable(ped_pomdp src/PedPomdpNode.cpp src/controller.cpp)
rosbuild_add_compile_flags(ped_pomdp ${PED_COMPILE_FLAGS}) 
rosbuild_add_compile_flags(ped_pomdp ${WARNING_FLAGS})

target_link_libraries(ped_pomdp despot)

#rosbuild_add_executable(pomcp-test src/pomcp/src/executer.cpp)
#rosbuild_add_compile_flags(pomcp-test ${WARNING_FLAGS})
#target_link_libraries(pomcp-test pomcp)

rosbuild_add_executable(local_frame src/local_frame.cpp src/planner/param.cpp)
rosbuild_add_compile_flags(local_frame ${PED_COMPILE_FLAGS})

rosbuild_add_executable(vel_publisher src/VelPublisher.cpp)


