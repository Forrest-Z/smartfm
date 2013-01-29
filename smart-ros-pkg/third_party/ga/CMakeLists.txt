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

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/GA1DArrayGenome.C src/GA1DBinStrGenome.C src/GA2DArrayGenome.C src/GA2DBinStrGenome.C src/GA3DArrayGenome.C src/GA3DBinStrGenome.C src/GAAllele.C src/GABaseGA.C src/GABin2DecGenome.C src/gabincvt.C src/GABinStr.C src/GADCrowdingGA.C src/GADemeGA.C src/gaerror.C src/GAGenome.C src/GAIncGA.C src/GAListBASE.C src/GAList.C src/GAListGenome.C src/GAParameter.C src/GAPopulation.C src/garandom.C src/GARealGenome.C src/GAScaling.C src/GASelector.C src/GASimpleGA.C src/GASStateGA.C src/GAStatistics.C src/GAStringGenome.C src/GATreeBASE.C src/GATree.C src/GATreeGenome.C)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
