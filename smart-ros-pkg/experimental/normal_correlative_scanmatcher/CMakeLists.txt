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
set(ISAM_LIBS isam cholmod suitesparseconfig colamd amd lapack blas rt)
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(/usr/include/vtk-5.8)
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES( "/usr/include/mysql" )
rosbuild_add_executable(RasterMapImageOpt src/RasterMapImageOpt.cpp src/dbgstream.cpp)
target_link_libraries(RasterMapImageOpt  ${OpenCV_LIBS} vtkCommon vtkRendering vtkFiltering mysqlpp)
rosbuild_add_openmp_flags(RasterMapImageOpt)
rosbuild_add_executable(allmatching src/all_matching.cpp src/dbgstream.cpp)
target_link_libraries(allmatching ${OpenCV_LIBS} mysqlpp)
rosbuild_add_openmp_flags(allmatching)
rosbuild_add_executable(isam_full src/isam_full.cpp src/dbgstream.cpp)
target_link_libraries(isam_full ${ISAM_LIBS} ${OpenCV_LIBS} mysqlpp)
#rosbuild_add_executable(marker_test src/marker_test.cpp)
#rosbuild_add_executable(normals_matching_pso src/normals_matching_pso.cpp src/dbgstream.cpp)
#target_link_libraries(normals_matching_pso  ${OpenCV_LIBS} vtkCommon vtkRendering vtkFiltering)
#rosbuild_add_executable(GeneticOpt src/GeneticOpt.cpp src/dbgstream.cpp)
#target_link_libraries(GeneticOpt  ${OpenCV_LIBS} vtkCommon vtkRendering vtkFiltering)
#rosbuild_add_executable(ga_ex9 src/ga_ex9.cpp src/dbgstream.cpp)
#target_link_libraries(ga_ex9  ${OpenCV_LIBS} vtkCommon vtkRendering vtkFiltering)
#rosbuild_add_executable(correlative_kdtree src/correlative_kdtree.cpp)
