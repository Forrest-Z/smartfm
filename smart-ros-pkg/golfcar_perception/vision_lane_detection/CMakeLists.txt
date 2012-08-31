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

rosbuild_add_library(image_proc src/image_proc.cpp src/libsvm/svm.cpp)
rosbuild_add_executable(ipm src/ipm.cpp)
target_link_libraries(ipm ${OpenCV_LIBRARIES} image_proc)
rosbuild_link_boost(ipm signals)
                    
rosbuild_add_executable(addCamInfo src/addCamInfo.cpp)
target_link_libraries(addCamInfo ${OpenCV_LIBRARIES})

rosbuild_add_executable(experimental_rgb src/experimental_rgb.cpp)
target_link_libraries(experimental_rgb ${OpenCV_LIBRARIES})
rosbuild_link_boost(experimental_rgb signals)

#rosbuild_add_executable(conti_lane src/conti_lane.cpp)
#target_link_libraries(conti_lane ${OpenCV_LIBRARIES})

rosbuild_add_executable(single_pic src/single_pic.cpp)
target_link_libraries(single_pic ${OpenCV_LIBRARIES})

rosbuild_add_executable(model_geometry src/model_geometry.cpp)
target_link_libraries(model_geometry ${OpenCV_LIBRARIES})

rosbuild_add_executable(generate_training_data src/generate_training_data.cpp)
target_link_libraries(generate_training_data ${OpenCV_LIBRARIES})

#rosbuild_add_executable(test_eigen src/test_eigen.cpp)
#target_link_libraries(test_eigen ${OpenCV_LIBRARIES})
rosbuild_add_executable(data_formating src/data_formating.cpp)
rosbuild_add_executable(extrinsic_calibration src/extrinsic_calibration.cpp)


#target_link_libraries(wrap_example ${OpenCV_LIBRARIES})
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
