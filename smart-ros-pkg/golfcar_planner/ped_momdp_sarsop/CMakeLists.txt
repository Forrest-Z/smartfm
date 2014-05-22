cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries #set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set(CMAKE_C_COMPILER "gcc-4.8")
#set(CMAKE_CXX_COMPILER "g++")
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# search header files in those sub directories
# Maybe a glob could be possible
INCLUDE_DIRECTORIES(

	#src/pomcp/src
	src/despot_multiple/src
	#src/despot_multiple/src/belief_update
	#src/despot_multiple/src/lower_bound
	#src/despot_multiple/src/upper_bound
	#src/despot_multiple/src/problems/pedestrian_changelane

)


#SET(LIBPOMCP_SRCS
#    src/pomcp/src/battleship.cpp   
# 
#    src/pomcp/src/simulator.cpp
#    src/pomcp/src/beliefstate.cpp 
#    src/pomcp/src/mcts.cpp       
#    src/pomcp/src/pedestrian_dynamic.cpp
#    src/pomcp/src/pedestrian_changelane.cpp
#    src/pomcp/src/tag.cpp
#    src/pomcp/src/coord.cpp   
#    src/pomcp/src/network.cpp    
#    src/pomcp/src/testsimulator.cpp
#    #src/pomcp/src/executer.cpp 
#    src/pomcp/src/node.cpp      
#    src/pomcp/src/pocman.cpp  
#    src/pomcp/src/utils.cpp
#    #src/pomcp/src/experiment.cpp 
#    src/pomcp/src/pedestrian.cpp 
#    src/pomcp/src/rocksample.cpp
#    src/pomcp/src/SFM.cpp
#    src/pomcp/src/window.cpp
#    src/pomcp/src/map.cpp
#    src/pomcp/src/math_utils.cpp
#)


SET(LIBDESPOT_SRCS
	src/despot_multiple/src/problems/pedestrian_changelane/pedestrian_state.cpp
	src/despot_multiple/src/globals.cpp
	src/despot_multiple/src/random_streams.cpp
    src/despot_multiple/src/problems/pedestrian_changelane/SFM.cpp
    src/despot_multiple/src/problems/pedestrian_changelane/window.cpp
    src/despot_multiple/src/problems/pedestrian_changelane/map.cpp
    src/despot_multiple/src/problems/pedestrian_changelane/math_utils.cpp
	src/despot_multiple/src/problems/pedestrian_changelane/coord.cpp
)
# There are too many compiler warnings with problem-solver code, so let's disable some:
set(WARNING_FLAGS "-Wno-sign-compare -Wno-unused-variable -Wno-ignored-qualifiers -Wno-write-strings")

add_definitions(-DDEBUG_LOG_ON)





#rosbuild_add_library(pomcp ${LIBPOMCP_SRCS})
rosbuild_add_library(despot ${LIBDESPOT_SRCS})
#rosbuild_add_compile_flags(despot -std=c++0x -O0 -ggdb  -fbounds-check)
rosbuild_add_compile_flags(despot -std=c++0x -O3)
rosbuild_add_executable(ped-momdp-sarsop src/pedestrian_momdp_realPed.cpp src/momdp.cpp)
#rosbuild_add_compile_flags(ped-momdp-sarsop -std=c++0x -O0 -ggdb -fbounds-check)
rosbuild_add_compile_flags(ped-momdp-sarsop -std=c++0x -O3)
rosbuild_add_compile_flags(ped-momdp-sarsop ${WARNING_FLAGS})

target_link_libraries(ped-momdp-sarsop despot  tinyxml)

#rosbuild_add_executable(pomcp-test src/pomcp/src/executer.cpp)
#rosbuild_add_compile_flags(pomcp-test ${WARNING_FLAGS})
#target_link_libraries(pomcp-test pomcp)

rosbuild_add_executable(local_frame src/local_frame.cpp)
rosbuild_add_compile_flags(local_frame -std=c++0x)

rosbuild_add_executable(vel_publisher src/VelPublisher.cpp)

rosbuild_add_executable(path_planner src/PathPlanner.cpp)

