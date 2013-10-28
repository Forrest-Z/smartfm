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

# search header files in those sub directories
# Maybe a glob could be possible
INCLUDE_DIRECTORIES(
    src/problem-solver
    src/problem-solver/Core
    src/problem-solver/Models/MOMDP
    src/problem-solver/Bounds
    src/problem-solver/Algorithms
    src/problem-solver/Algorithms/SARSOP
    src/problem-solver/MathLib
    src/problem-solver/Parser
    src/problem-solver/Parser/Cassandra
    src/problem-solver/Parser/POMDPX
    src/problem-solver/Utils
    src/problem-solver/OfflineSolver
    src/problem-solver/Evaluator
    src/problem-solver/Simulator
)

# source files for the appl lib
# Maybe a glob could be possible
SET(LIBAPPL_SRCS
    src/problem-solver/PolicyGraph/PolicyGraph.cpp
    src/problem-solver/PolicyGraph/PolicyGraphGenerator.cpp
    src/problem-solver/Evaluator/EvaluationEngine.cpp
    src/problem-solver/Evaluator/EvaluatorSampleEngine.cpp
    src/problem-solver/Simulator/PrintTuple.cpp
    src/problem-solver/Simulator/SimulationEngine.cpp
    src/problem-solver/Simulator/SimulationEngine2.cpp
    src/problem-solver/MathLib/DenseVector.cpp
    src/problem-solver/MathLib/MathLib.cpp
    src/problem-solver/MathLib/SparseMatrix.cpp
    src/problem-solver/MathLib/SparseVector.cpp
    src/problem-solver/Algorithms/SARSOP/BinManager.cpp
    src/problem-solver/Algorithms/SARSOP/BinManagerSet.cpp
    src/problem-solver/Algorithms/SARSOP/Sample.cpp
    src/problem-solver/Algorithms/SARSOP/SampleBP.cpp
    src/problem-solver/Algorithms/SARSOP/SARSOP.cpp
    src/problem-solver/Algorithms/SARSOP/SARSOPPrune.cpp
    src/problem-solver/Models/MOMDP/StateTransitionX.cpp
    src/problem-solver/Models/MOMDP/IVariable.cpp
    src/problem-solver/Models/MOMDP/BooleanVariable.cpp
    src/problem-solver/Models/MOMDP/BeliefTransitionMOMDPLite.cpp
    src/problem-solver/Models/MOMDP/Variable.cpp
    src/problem-solver/Models/MOMDP/IVariableValue.cpp
    src/problem-solver/Models/MOMDP/StateTransitionXY.cpp
    src/problem-solver/Models/MOMDP/MOMDP.cpp
    src/problem-solver/Models/MOMDP/StateTransitionXXpY.cpp
    src/problem-solver/Models/MOMDP/VariableRelation.cpp
    src/problem-solver/Models/MOMDP/StateTransitionY.cpp
    src/problem-solver/Models/MOMDP/VariableValue.cpp
    src/problem-solver/Models/MOMDP/VariableCombined.cpp
    src/problem-solver/Models/MOMDP/MOMDPLite.cpp
    src/problem-solver/Models/MOMDP/Rewards.cpp
    src/problem-solver/Models/MOMDP/ObservationProbabilities.cpp
    src/problem-solver/Models/MOMDP/BeliefTransitionMOMDP.cpp
    src/problem-solver/OfflineSolver/GlobalResource.cpp
    src/problem-solver/OfflineSolver/solverUtils.cpp
    src/problem-solver/Bounds/AlphaPlane.cpp
    src/problem-solver/Bounds/AlphaPlanePool.cpp
    src/problem-solver/Bounds/AlphaPlanePoolSet.cpp
    src/problem-solver/Bounds/AlphaVectorPolicy.cpp
    src/problem-solver/Bounds/BackupAlphaPlaneMOMDP.cpp
    src/problem-solver/Bounds/BackupAlphaPlaneMOMDPLite.cpp
    src/problem-solver/Bounds/BackupBeliefValuePairMOMDP.cpp
    src/problem-solver/Bounds/BackupBeliefValuePairMOMDPLite.cpp
    src/problem-solver/Bounds/BeliefValuePair.cpp
    src/problem-solver/Bounds/BeliefValuePairPool.cpp
    src/problem-solver/Bounds/BeliefValuePairPoolSet.cpp
    src/problem-solver/Bounds/BlindLBInitializer.cpp
    src/problem-solver/Bounds/FastInfUBInitializer.cpp
    src/problem-solver/Bounds/FullObsUBInitializer.cpp
    src/problem-solver/Bounds/PruneAlphaPlane.cpp
    src/problem-solver/Bounds/PruneBeliefValuePair.cpp
    src/problem-solver/Bounds/xml_parse_lib.c
    src/problem-solver/Core/Actions.cpp
    src/problem-solver/Core/VariableContainer.cpp
    src/problem-solver/Core/Belief.cpp
    src/problem-solver/Core/BeliefCache.cpp
    src/problem-solver/Core/BeliefException.cpp
    src/problem-solver/Core/BeliefForest.cpp
    src/problem-solver/Core/BeliefTreeNode.cpp
    src/problem-solver/Core/BeliefWithState.cpp
    src/problem-solver/Core/MObject.cpp
    src/problem-solver/Core/Observations.cpp
    src/problem-solver/Core/States.cpp
    src/problem-solver/Core/UniqueBeliefHeap.cpp
    src/problem-solver/Parser/Cassandra/Parser.cpp
    src/problem-solver/Parser/Cassandra/POMDP.cpp
    src/problem-solver/Parser/Cassandra/pomdpCassandraWrapper.cpp
    src/problem-solver/Parser/ParserSelector.cpp
    src/problem-solver/Parser/POMDPX/FactoredPomdp.cpp
    src/problem-solver/Parser/POMDPX/Function.cpp
    src/problem-solver/Parser/POMDPX/ObsAct.cpp
    src/problem-solver/Parser/POMDPX/oldPOMDP.cpp
    src/problem-solver/Parser/POMDPX/PreCEntry.cpp
    src/problem-solver/Parser/POMDPX/PreCMatrix.cpp
    src/problem-solver/Parser/POMDPX/SparseEntry.cpp
    src/problem-solver/Parser/POMDPX/SparseTable.cpp
    src/problem-solver/Parser/POMDPX/State.cpp
    src/problem-solver/Parser/POMDPX/StateObsAct.cpp
    src/problem-solver/Utils/InfoLog.cpp
    src/problem-solver/Utils/md5.cpp
    src/problem-solver/Utils/md5wrapper.cpp
    src/problem-solver/Utils/CPMemUtils.cpp
    src/problem-solver/Utils/StatsCollector.cpp
    src/problem-solver/Utils/SimulationRewardCollector.cpp
    src/problem-solver/Parser/Cassandra/include/pomdp_spec.tab.cc
    src/problem-solver/Parser/Cassandra/include/pomdp_spec.yy.cc
    src/problem-solver/miniposix/getopt.c
    src/problem-solver/miniposix/getopt1.c
    src/problem-solver/miniposix/getopt_init.c
    src/problem-solver/Parser/Cassandra/decision-tree.c
    src/problem-solver/Parser/Cassandra/imm-reward.c
    src/problem-solver/Parser/Cassandra/mdpCassandra.c
    src/problem-solver/Parser/Cassandra/parse_err.c
    src/problem-solver/Parser/Cassandra/parse_hash.c
    src/problem-solver/Parser/Cassandra/sparse-matrix.c
)

# There are too many compiler warnings with problem-solver code, so let's disable some:
set(WARNING_FLAGS "-Wno-sign-compare -Wno-unused-variable -Wno-ignored-qualifiers -Wno-write-strings")

add_definitions(-DDEBUG_LOG_ON)

rosbuild_add_library(appl ${LIBAPPL_SRCS})
rosbuild_add_compile_flags(appl ${WARNING_FLAGS} -msse2 -mfpmath=sse)

rosbuild_add_executable(pomdpeval src/problem-solver/Evaluator/Evaluator.cpp)
rosbuild_add_compile_flags(pomdpeval ${WARNING_FLAGS})
target_link_libraries(pomdpeval appl tinyxml)

rosbuild_add_executable(pomdpsim src/problem-solver/Simulator/Simulator.cpp)
rosbuild_add_compile_flags(pomdpsim ${WARNING_FLAGS})
target_link_libraries(pomdpsim appl tinyxml)

rosbuild_add_executable(pomdpsim-rss src/problem-solver/Simulator/Simulator2.cpp)
rosbuild_add_compile_flags(pomdpsim-rss ${WARNING_FLAGS})
target_link_libraries(pomdpsim-rss appl tinyxml)

rosbuild_add_executable(pomdpsol src/problem-solver/OfflineSolver/solver.cpp)
rosbuild_add_compile_flags(pomdpsol ${WARNING_FLAGS})
target_link_libraries(pomdpsol appl tinyxml)

rosbuild_add_executable(polgraph src/problem-solver/PolicyGraph/PolicyGraph.cpp)
rosbuild_add_compile_flags(polgraph ${WARNING_FLAGS})
target_link_libraries(polgraph appl tinyxml)

rosbuild_add_executable(pomdpconvert src/problem-solver/PomdpConvertor/convertor.cpp)
rosbuild_add_compile_flags(pomdpconvert ${WARNING_FLAGS})
target_link_libraries(pomdpconvert appl tinyxml)

rosbuild_add_executable(Tjunc-momdp-sarsop src/Vehicle_momdp_realPed.cpp src/ROS_SimulationEngine.cpp src/momdp.cpp)
rosbuild_add_compile_flags(Tjunc-momdp-sarsop ${WARNING_FLAGS})
target_link_libraries(Tjunc-momdp-sarsop appl tinyxml)

rosbuild_add_executable(Tjunc-gap30-realveh src/Vehicle_momdp_realPed_realveh.cpp src/ROS_SimulationEngine.cpp src/gap30_realveh.cpp)
rosbuild_add_compile_flags(Tjunc-gap30-realveh ${WARNING_FLAGS})
target_link_libraries(Tjunc-gap30-realveh appl tinyxml)

rosbuild_add_executable(Tjunc-momdp-realveh src/Vehicle_momdp_realPed_realveh.cpp src/ROS_SimulationEngine.cpp src/momdp_realveh.cpp)
rosbuild_add_compile_flags(Tjunc-momdp-realveh ${WARNING_FLAGS})
target_link_libraries(Tjunc-momdp-realveh appl tinyxml)

rosbuild_add_executable(Tjunc-GAP0 src/Vehicle_momdp_realPed.cpp src/ROS_SimulationEngine.cpp src/gap0.cpp)
rosbuild_add_compile_flags(Tjunc-GAP0 ${WARNING_FLAGS})
target_link_libraries(Tjunc-GAP0 appl tinyxml)

rosbuild_add_executable(Tjunc-GAP1 src/Vehicle_momdp_realPed.cpp src/ROS_SimulationEngine.cpp src/gap1.cpp)
rosbuild_add_compile_flags(Tjunc-GAP1 ${WARNING_FLAGS})
target_link_libraries(Tjunc-GAP1 appl tinyxml)

rosbuild_add_executable(Tjunc-GAP2 src/Vehicle_momdp_realPed.cpp src/ROS_SimulationEngine.cpp src/gap2.cpp)
rosbuild_add_compile_flags(Tjunc-GAP2 ${WARNING_FLAGS})
target_link_libraries(Tjunc-GAP2 appl tinyxml)

rosbuild_add_executable(Tjunc-GAP3 src/Vehicle_momdp_realPed.cpp src/ROS_SimulationEngine.cpp src/gap3.cpp)
rosbuild_add_compile_flags(Tjunc-GAP3 ${WARNING_FLAGS})
target_link_libraries(Tjunc-GAP3 appl tinyxml)

rosbuild_add_executable(Tjunc_local_frame src/local_frame.cpp)
rosbuild_add_compile_flags(Tjunc_local_frame -std=c++0x)
