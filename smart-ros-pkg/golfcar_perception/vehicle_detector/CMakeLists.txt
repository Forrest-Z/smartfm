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


# NOTE: It's not necessary to search for OpenCV or even link with OpenCV:
# this package depends on cv_bridge, so it will automatically be linked.
# However we can look for CUDA, and compile the classifier only if CUDA is
# present, and link with opencv_gpu

find_package(OpenCV REQUIRED)
rosbuild_add_boost_directories()

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

#rosbuild_add_executable(pedestrian_detector_HOG src/pedestrian_detector_HOG.cpp)

#Note: opencv gpu only works when all libopencv is overwritten from the
#CUDA compiled source to /opt/ros/fuerte/lib. If not precomp.cpp will complain

rosbuild_add_executable(vehicle_detection src/vehicle_detection.cpp)
target_link_libraries(vehicle_detection ${OpenCV_LIBS})
rosbuild_link_boost(vehicle_detection signals)

rosbuild_add_executable(mergevec src/mergevec.cpp src/apps/cvboost.cpp src/apps/cvcommon.cpp src/apps/cvsamples.cpp src/apps/cvhaarclassifier.cpp src/apps/cvhaartraining.cpp)
target_link_libraries(mergevec ${OpenCV_LIBS})

rosbuild_add_executable(vehicle_detector src/vehicle_haar.cpp)
target_link_libraries(vehicle_detector ${OpenCV_LIBS})
rosbuild_link_boost(vehicle_detector signals)
