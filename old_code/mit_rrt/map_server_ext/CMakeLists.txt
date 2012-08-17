cmake_minimum_required(VERSION 2.4.6 FATAL_ERROR)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_add_library(image_loader src/image_loader.cpp)
target_link_libraries(image_loader SDL SDL_image)

rosbuild_add_executable(bin/map_server src/main.cpp)
target_link_libraries(bin/map_server image_loader SDL SDL_image yaml-cpp)


rosbuild_add_executable(bin/map_saver src/map_saver.cpp)

