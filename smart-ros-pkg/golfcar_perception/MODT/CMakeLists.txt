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
rosbuild_add_boost_directories()
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(OpenCV REQUIRED)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(verti_laser_sickldmrs src/verti_laser_sickldmrs.cpp)
rosbuild_link_boost(verti_laser_sickldmrs signals)

rosbuild_add_executable(DATMO src/DATMO.cpp)
target_link_libraries(DATMO ${OpenCV_LIBRARIES})
rosbuild_link_boost(DATMO signals)

rosbuild_add_executable(DATMO_recordData src/DATMO_recordData.cpp)
target_link_libraries(DATMO_recordData ${OpenCV_LIBRARIES})
rosbuild_link_boost(DATMO_recordData signals)

rosbuild_add_executable(DATMO_labellingData src/DATMO_labellingData.cpp)
target_link_libraries(DATMO_labellingData ${OpenCV_LIBRARIES})
rosbuild_link_boost(DATMO_labellingData signals)

rosbuild_add_executable(DATMO_Ped src/DATMO_Ped.cpp)
target_link_libraries(DATMO_Ped ${OpenCV_LIBRARIES})
rosbuild_link_boost(DATMO_Ped signals)

rosbuild_add_executable(DATMO_derivedLabelling src/DATMO_derivedLabelling.cpp)
target_link_libraries(DATMO_derivedLabelling ${OpenCV_LIBRARIES})
rosbuild_link_boost(DATMO_derivedLabelling signals)

rosbuild_add_executable(DATMO_dataformatting src/DATMO_dataformatting.cpp)
target_link_libraries(DATMO_dataformatting ${OpenCV_LIBRARIES})
rosbuild_link_boost(DATMO_dataformatting signals)

rosbuild_add_executable(DATMO_online src/DATMO_online.cpp)
target_link_libraries(DATMO_online ${OpenCV_LIBRARIES})
rosbuild_link_boost(DATMO_online signals)

rosbuild_add_executable(DATMO_CandGen src/DATMO_CandGen.cpp)
target_link_libraries(DATMO_CandGen ${OpenCV_LIBRARIES})
rosbuild_link_boost(DATMO_CandGen signals)

rosbuild_add_executable(supervised_segmentation src/supervised_segmentation.cpp)
target_link_libraries(supervised_segmentation ${OpenCV_LIBRARIES})
rosbuild_link_boost(supervised_segmentation signals)

#rosbuild_add_executable(graph_segmentation src/segmentation/segment.cpp src/segmentation/segment-image.h src/segmentation/segment-graph.h src/segmentation/disjoint-set.h)
rosbuild_add_executable(graph_segmentation src/segmentation/segment.cpp)

rosbuild_add_library(shape_analyzer src/RANSAC_model/shape_analyzer.cpp)
rosbuild_add_executable(ransac_node src/RANSAC_model/ransac_node.cpp)
target_link_libraries(ransac_node shape_analyzer)

rosbuild_add_executable(spatial_temporal_segmentation src/spatial_temporal_segmentation.cpp)
rosbuild_link_boost(spatial_temporal_segmentation signals)
target_link_libraries(spatial_temporal_segmentation ${OpenCV_LIBRARIES})

rosbuild_add_executable(ST_autolabelling src/ST_autolabelling.cpp)
rosbuild_link_boost(ST_autolabelling signals)
target_link_libraries(ST_autolabelling ${OpenCV_LIBRARIES})

rosbuild_add_executable(ST_GenRawTrainingData src/ST_GenRawTrainingData.cpp)
rosbuild_link_boost(ST_GenRawTrainingData signals)
target_link_libraries(ST_GenRawTrainingData ${OpenCV_LIBRARIES})

rosbuild_add_executable(ST_LabelRawTrainingData src/ST_LabelRawTrainingData.cpp)
rosbuild_link_boost(ST_LabelRawTrainingData signals)
target_link_libraries(ST_LabelRawTrainingData ${OpenCV_LIBRARIES})

rosbuild_add_executable(simple_moving_check src/simple_moving_check.cpp)
rosbuild_link_boost(simple_moving_check signals)
target_link_libraries(simple_moving_check ${OpenCV_LIBRARIES})

rosbuild_add_executable(vehicle_pose_estimation src/vehicle_pose_estimation.cpp)

rosbuild_add_executable(vehicle_tracking_ICP src/vehicle_tracking_ICP.cpp src/EKF_tracker/ConstSpeed_EKF_tracker.cpp src/EKF_tracker/nonlinearanalyticconditionalgaussianodo.cpp)
target_link_libraries(vehicle_tracking_ICP ${OpenCV_LIBRARIES})

rosbuild_add_executable(vehicle_tracking_box src/vehicle_tracking_box.cpp src/EKF_tracker/ConstSpeed_EKF_tracker.cpp src/EKF_tracker/nonlinearanalyticconditionalgaussianodo.cpp)
target_link_libraries(vehicle_tracking_box ${OpenCV_LIBRARIES})

#rosbuild_add_executable(vehicle_tracking_gridsearch src/vehicle_tracking_gridsearch.cpp)
#rosbuild_add_executable(ICP_example src/ICP_example.cpp)
