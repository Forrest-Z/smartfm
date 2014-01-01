cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Declare a program which uses MRPT: 
# Since we declared a ROS dependencies to mrpt_libs, 
# there's no need to add further header/lib deps at this point
#rosbuild_add_library(${PROJECT_NAME} src/ransac_Lshape.cpp)
rosbuild_add_executable(ransac_example src/ransac_node.cpp)
