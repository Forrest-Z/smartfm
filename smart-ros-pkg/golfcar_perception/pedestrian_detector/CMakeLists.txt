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
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(pedestrian_detector_HOG
			#src/pedestrian_detector_HOG.cpp)
rosbuild_add_executable(video src/video.cpp)
rosbuild_add_executable(cv src/cv.cpp)
rosbuild_add_executable(hog_gpu src/hog_gpu.cpp)
#rosbuild_add_executable(hog src/hog.cpp)
rosbuild_add_executable(rect_talker src/rect_talker.cpp)
rosbuild_add_executable(classifier src/classifier.cpp)
rosbuild_add_executable(people_detect_sensor src/people_detect_sensor.cpp)
rosbuild_add_executable(classifier_cpu src/classifier_cpu.cpp)
rosbuild_add_executable(hog_cpu src/hog_cpu.cpp)
rosbuild_add_executable(visualizer src/visualizer.cpp)
rosbuild_add_executable(visualizer_320 src/visualizer_320.cpp)
rosbuild_add_link_flags(visualizer_320 -lopencv_gpu)
rosbuild_add_link_flags(visualizer -lopencv_gpu)
rosbuild_add_link_flags(hog_cpu -lopencv_gpu)
#rosbuild_add_link_flags(hog -lopencv_gpu)
rosbuild_add_link_flags(hog_gpu -lopencv_gpu)
rosbuild_add_link_flags(cv -lopencv_gpu)
rosbuild_add_link_flags(classifier -lopencv_gpu)
rosbuild_add_link_flags(classifier_cpu -lopencv_gpu)
rosbuild_add_link_flags(video -lopencv_gpu)
rosbuild_add_link_flags(people_detect_sensor -lopencv_gpu)
rosbuild_add_link_flags(rect_talker -lopencv_gpu)
