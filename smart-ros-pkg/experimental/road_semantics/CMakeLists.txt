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
find_package( PkgConfig REQUIRED)
pkg_check_modules( gsl REQUIRED gsl)

#rosbuild_add_library(evg-thin src/evg/evg-thin.cc)
#target_link_libraries(evg-thin ${OpenCV_LIBRARIES})
#rosbuild_add_executable(example src/evg/example.cc)
#target_link_libraries(example evg-thin)

rosbuild_add_library(spline_fitting src/spline_fitting/spline_fitting.cpp )
target_link_libraries( spline_fitting ${gsl_LIBRARIES} )

rosbuild_add_library(topo_extractor src/topo_extractor/TopoExtractor.cpp)
target_link_libraries(topo_extractor ${OpenCV_LIBRARIES} spline_fitting)
rosbuild_add_library(topo_semantic src/topo_semantic/TopoSemantic.cpp)
target_link_libraries(topo_semantic ${OpenCV_LIBRARIES})

rosbuild_add_library (road_semantics src/road_semantics.cpp)
target_link_libraries(road_semantics topo_extractor topo_semantic ${OpenCV_LIBRARIES})

rosbuild_add_executable(road_semantics_node src/road_semantics_node.cpp)
target_link_libraries(road_semantics_node road_semantics)

add_executable( gsl_example src/spline_fitting/gsl_example.cpp )
target_link_libraries( gsl_example ${gsl_LIBRARIES} )


rosbuild_add_boost_directories()
rosbuild_add_executable(test_BGL src/test_BGL.cpp)
rosbuild_link_boost(test_BGL graph)
