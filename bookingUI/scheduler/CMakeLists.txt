cmake_minimum_required (VERSION 2.6)
project (Scheduler)

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)

set(STATION_PATH_DIR "../../smart-ros-pkg/golfcar_navigation/golfcar_route_planner")
include_directories("${STATION_PATH_DIR}/include/golfcar_route_planner")
LINK_DIRECTORIES( ${LINK_DIRECTORIES} "${STATION_PATH_DIR}/lib" )

add_executable(scheduler src/Socket.cpp src/ClientSocket.cpp src/ServerSocket.cpp src/Scheduler.cpp src/SchedulerUI.cpp src/SchedulerTalker.cpp src/VehicleTalker.cpp src/SchedulerMain.cpp)
target_link_libraries(scheduler stationpath)