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

rosbuild_add_boost_directories()

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/ccrrts.hpp src/cc_system.cpp src/risk_evaluate.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(constraint_process src/ChanceConstraint.cpp)
rosbuild_link_boost(constraint_process signals)

rosbuild_add_executable(sim_obst src/simulation_cc.cpp)

rosbuild_add_executable(obst_filt src/obst_filter.cpp)

#rosbuild_add_executable(risk_evaluate src/risk_evaluate.cpp)

rosbuild_add_executable(ccrrts src/ccrrts_node.cpp)

rosbuild_add_executable(norm_ccrrts src/norm_rrts_node.cpp src/norm_ccrrts.hpp src/norm_system.cpp src/risk_evaluate.cpp)

rosbuild_add_executable(simple_goal src/simple_goal.cpp)

#target_link_libraries(example ${PROJECT_NAME})
