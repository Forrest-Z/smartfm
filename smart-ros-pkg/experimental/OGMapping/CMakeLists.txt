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

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


find_package(OpenCV REQUIRED)

rosbuild_add_boost_directories()
rosbuild_add_executable(OGMapper src/OGMapper.cpp)
target_link_libraries(OGMapper SDL SDL_image yaml-cpp)
rosbuild_link_boost(OGMapper signals)

rosbuild_add_executable(save_pub src/save_pub.cpp)

rosbuild_add_executable(skeletonization src/skeletonization.cpp)
target_link_libraries(skeletonization ${OpenCV_LIBRARIES})

rosbuild_add_executable(distance_transform src/distance_transform.cpp)
target_link_libraries(distance_transform ${OpenCV_LIBRARIES})

rosbuild_add_executable(delaunay src/delaunay.cpp)
target_link_libraries(delaunay ${OpenCV_LIBRARIES})


#target_link_libraries(wrap_example ${OpenCV_LIBRARIES})
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
