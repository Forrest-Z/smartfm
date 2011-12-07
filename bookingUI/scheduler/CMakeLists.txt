cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0015 NEW)
project (Scheduler)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
FIND_PACKAGE(MySQLCppConn REQUIRED)

# boost/variant.hpp required by cppconn/driver.h
# Install it from debian package libboost-dev
FIND_PACKAGE(Boost REQUIRED)
IF (BOOST_FOUND)
    INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})
    ADD_DEFINITIONS( "-DHAS_BOOST" )
ENDIF()

set(CMAKE_CXX_FLAGS "-g -Wall")

set(STATION_PATH_DIR ../../smart-ros-pkg/golfcar_navigation/golfcar_route_planner)
include_directories(${STATION_PATH_DIR}/include/golfcar_route_planner)
link_directories( ${STATION_PATH_DIR}/lib )

add_executable(MySQLConnectorC++Client src/MySQLConnectorC++Client.cpp)
target_link_libraries(MySQLConnectorC++Client mysqlcppconn)

add_executable(scheduler src/Scheduler.cpp src/DBTalker.cpp src/SchedulerUI.cpp src/SchedulerMain.cpp)
target_link_libraries(scheduler stationpath ncurses mysqlcppconn)
