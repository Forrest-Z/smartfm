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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
INCLUDE_DIRECTORIES(src/problem-solver src/problem-solver/Core src/problem-solver/Models/MOMDP src/problem-solver/Bounds src/problem-solver/Algorithms src/problem-solver/Algorithms/SARSOP src/problem-solver/MathLib src/problem-solver/Parser src/problem-solver/Parser/Cassandra src/problem-solver/Parser/POMDPX src/problem-solver/Utils src/problem-solver/OfflineSolver)

rosbuild_add_executable(ped-momdp-sarsop src/pedestrian_momdp_realPed.cpp src/PSG_SimulationEngine.cpp)

# There are too many compiler warnings with problem-solver code, so let's disable some:
rosbuild_add_compile_flags(ped-momdp-sarsop -Wno-unused-variable -Wno-ignored-qualifiers)

# In order to link with libappl, first build it and install it:
#    cd src/problem-solver
#    make
#    sudo cp libappl.a /usr/local/lib
target_link_libraries(ped-momdp-sarsop appl tinyxml)
