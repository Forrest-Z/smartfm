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
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/ReflexxesAPI.cpp src/ReflexxesOutputValuesToFile.cpp src/TypeIVRMLABK.cpp src/TypeIVRMLCalculatePositionalExtrems.cpp src/TypeIVRMLCalculateScalingVector.cpp src/TypeIVRMLDecisionTree1A.cpp src/TypeIVRMLDecisionTree1B1.cpp src/TypeIVRMLDecisionTree1B2.cpp src/TypeIVRMLDecisionTree1B3.cpp src/TypeIVRMLDecisionTree1C.cpp src/TypeIVRMLDecisionTree2.cpp src/TypeIVRMLFallBackStrategy.cpp src/TypeIVRMLIsPhaseSynchronizationPossible.cpp src/TypeIVRMLMath.cpp src/TypeIVRMLPolynomial.cpp src/TypeIVRMLPosition.cpp src/TypeIVRMLQuicksort.cpp src/TypeIVRMLSetupModifiedSelectionVector.cpp src/TypeIVRMLStep1.cpp src/TypeIVRMLStep1Decisions.cpp src/TypeIVRMLStep1IntermediateChangingProfiles.cpp src/TypeIVRMLStep1IntermediateTimeProfiles.cpp src/TypeIVRMLStep1Profiles.cpp src/TypeIVRMLStep1RootFunctions.cpp src/TypeIVRMLStep2.cpp src/TypeIVRMLStep2Decisions.cpp src/TypeIVRMLStep2IntermediateProfiles.cpp src/TypeIVRMLStep2PhaseSynchronization.cpp src/TypeIVRMLStep2Profiles.cpp src/TypeIVRMLStep2RootFunctions.cpp src/TypeIVRMLStep2WithoutSynchronization.cpp src/TypeIVRMLStep3.cpp src/TypeIVRMLThreadFunction.cpp src/TypeIVRMLVelocityCalculatePositionalExtrems.cpp src/TypeIVRMLVelocity.cpp src/TypeIVRMLVelocityFallBackStrategy.cpp src/TypeIVRMLVelocityIsPhaseSynchronizationPossible.cpp src/TypeIVRMLVelocityMethods.cpp src/TypeIVRMLVelocitySetupPhaseSyncSelectionVector.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

