cmake_minimum_required (VERSION 2.8)
project (BookingCommon)

SET (CMAKE_CXX_FLAGS "-g -Wall")

INCLUDE_DIRECTORIES (src)

SET (LIB_SRCS src/Station.cpp src/StationDoesNotExist.cpp src/StationList.cpp src/StationPath.cpp src/Threaded.cpp src/RoutePlanner.cpp src/MissionComm.cpp)

ADD_LIBRARY (bookingcommon SHARED ${LIB_SRCS})
ADD_LIBRARY(bookingcommon-static STATIC ${LIB_SRCS})
SET_TARGET_PROPERTIES(bookingcommon-static PROPERTIES OUTPUT_NAME "bookingcommon")
SET_TARGET_PROPERTIES(bookingcommon-static PROPERTIES PREFIX "lib")

 # Help CMake 2.6.x and lower (not necessary for 2.8 and above, but doesn't hurt):
 SET_TARGET_PROPERTIES(bookingcommon PROPERTIES CLEAN_DIRECT_OUTPUT 1)
 SET_TARGET_PROPERTIES(bookingcommon-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)