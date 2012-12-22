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

find_package(OpenCV REQUIRED)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(matcher src/matcher.cpp)
#rosbuild_add_openmp_flags(matcher)

#rosbuild_add_executable(observationCache src/observationCache.cpp)
#rosbuild_link_boost(observationCache signals)
#rosbuild_add_openmp_flags(observationCache)

#target_link_libraries(example ${PROJECT_NAME})

set(ANN_SRC src/ANN/ANN.cpp src/ANN/brute.cpp src/ANN/kd_tree.cpp src/ANN/kd_util.cpp src/ANN/kd_split.cpp src/ANN/kd_dump.cpp src/ANN/kd_search.cpp src/ANN/kd_pr_search.cpp src/ANN/kd_fix_rad_search.cpp src/ANN/bd_tree.cpp src/ANN/bd_search.cpp src/ANN/bd_pr_search.cpp src/ANN/bd_fix_rad_search.cpp src/ANN/perf.cpp)
set(ISAM_LIBS isam cholmod suitesparseconfig colamd amd lapack blas rt)

#rosbuild_add_executable(ann_sample src/ann_sample.cpp ${ANN_SRC})
#rosbuild_add_executable(RasterMap src/RasterMap.cpp ${ANN_SRC})
#rosbuild_add_executable(RasterMapImage src/RasterMapImage.cpp ${ANN_SRC})
#rosbuild_add_executable(RasterMapImageOpt src/RasterMapImageOpt.cpp)

rosbuild_add_executable(readPtsAndMatch src/readPtsAndMatch.cpp src/dbgstream.cpp)
target_link_libraries(readPtsAndMatch  ${OpenCV_LIBS})
rosbuild_add_openmp_flags(readPtsAndMatch)

#rosbuild_add_executable(renderMap src/renderMap.cpp)

rosbuild_add_executable(RasterMapOdom src/RasterMapOdom.cpp)
target_link_libraries(RasterMapOdom  ${OpenCV_LIBS})
rosbuild_link_boost(RasterMapOdom signals)

rosbuild_add_executable(readScoreAndDisplay src/readScoreAndDisplay.cpp src/dbgstream.cpp)
target_link_libraries(readScoreAndDisplay  ${OpenCV_LIBS})


#target_link_libraries(renderMap  ${OpenCV_LIBS})

#target_link_libraries(RasterMap  ${OpenCV_LIBS})
#target_link_libraries(RasterMapImage  ${OpenCV_LIBS})
#target_link_libraries(RasterMapImageOpt  ${OpenCV_LIBS})


#rosbuild_add_openmp_flags(RasterMapImage)
#rosbuild_add_openmp_flags(RasterMapImageOpt)


#rosbuild_add_executable(isam_example src/isam_example.cpp)
#target_link_libraries(isam_example ${ISAM_LIBS})

#rosbuild_add_executable(isam_simple src/isam_simple.cpp)
#target_link_libraries(isam_simple ${ISAM_LIBS} ${OpenCV_LIBS})
#rosbuild_add_executable(random_test src/random_test.cpp)

rosbuild_add_executable(isam_full src/isam_full.cpp src/dbgstream.cpp)
target_link_libraries(isam_full ${ISAM_LIBS} ${OpenCV_LIBS})
rosbuild_add_openmp_flags(isam_full)

#rosbuild_add_executable(cairo_test src/cairo_test.cpp)
#target_link_libraries(cairo_test cairo)
#rosbuild_add_openmp_flags(cairo_test)

#rosbuild_add_executable(opengl_test src/opengl_test.cpp)
#target_link_libraries(opengl_test glut GLU)

#rosbuild_add_executable(dbgstream src/dbgstream.cpp)