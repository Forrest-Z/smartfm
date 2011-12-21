cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0015 NEW)
project (Scheduler)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_CXX_FLAGS "-g -Wall -O0")


set(CURSES_NEED_NCURSES TRUE)
FIND_PACKAGE(Curses REQUIRED)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
FIND_PACKAGE(MySQLCppConn REQUIRED)
IF(MySQLCppConn_FOUND)
    INCLUDE_DIRECTORIES(${MySQLCppConn_INCLUDE_DIR})
ENDIF()

# boost/variant.hpp required by cppconn/driver.h
# Install it from debian package libboost-dev
FIND_PACKAGE(Boost REQUIRED)
IF (BOOST_FOUND)
    INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})
ENDIF()

INCLUDE_DIRECTORIES(src)

SET( BookingCommonRootDir ../shared )
INCLUDE_DIRECTORIES(${BookingCommonRootDir}/src)
LINK_DIRECTORIES (${LINK_DIRECTORIES} ${BookingCommonRootDir})

add_executable(scheduler src/SchedulerTypes.cpp src/Vehicle.cpp src/Scheduler.cpp src/DBTalker.cpp src/SchedulerUI.cpp src/SchedulerMain.cpp)
target_link_libraries(scheduler ncurses mysqlcppconn bookingcommon)
