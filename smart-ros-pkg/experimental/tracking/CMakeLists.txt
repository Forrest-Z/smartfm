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

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
FIND_PACKAGE(GSL REQUIRED)
IF(GSL_FOUND)
    INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIR})
ENDIF()

INCLUDE_DIRECTORIES(/usr/local/include/bfl)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/test)

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


rosbuild_add_executable(test_gsl_spline src/test/test_gsl_spline.cpp)
target_link_libraries(test_gsl_spline ${GSL_LIBRARIES})

rosbuild_add_executable(test_path src/test/test_path.cpp src/test/Path.cpp)
target_link_libraries(test_path ${GSL_LIBRARIES})

rosbuild_add_executable(test_one_target src/test/test_one_target.cpp src/test/Path.cpp src/nonlinearanalyticconditionalgaussianmobile.cpp src/EKF.cpp src/Filter.cpp)
target_link_libraries(test_one_target ${GSL_LIBRARIES} orocos-bfl)
