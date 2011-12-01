cmake_minimum_required (VERSION 2.6)
project (Scheduler)

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)

add_executable(scheduler src/SchedulerMain.cpp src/SchedulerUI.cpp src/SchedulerTalker.cpp src/VehicleTalker.cpp src/Scheduler.cpp src/StationNetwork.cpp)