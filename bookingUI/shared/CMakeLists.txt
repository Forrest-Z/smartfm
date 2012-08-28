cmake_minimum_required (VERSION 2.8)
project (BookingCommon)

SET (CMAKE_CXX_FLAGS "-g -Wall")

# TinyXml is required. Since there was an API change at 2.6 we need to check
# the version number and define a flag accordingly
INCLUDE(FindPkgConfig)
pkg_check_modules(TINYXML REQUIRED tinyxml)
pkg_check_modules(TINYXML26 tinyxml>=2.6) #QUIET only works with cmake>=2.8.2
if( NOT TINYXML26_FOUND )
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTINYXML_API_PRE26")
    MESSAGE("Using old tinyxml API (pre 2.6). Setting the compilation flag TINYXML_API_PRE26.")
endif( NOT TINYXML26_FOUND )
INCLUDE_DIRECTORIES(${TINYXML_INCLUDE_DIRS})
LINK_DIRECTORIES(${TINYXML_LIBRARY_DIRS})

# We also need libcurl
FIND_PACKAGE(CURL REQUIRED)
INCLUDE_DIRECTORIES(${CURL_INCLUDE_DIRS})
LINK_DIRECTORIES(${CURL_LIBRARY_DIRS})

# boost/filesystem required to find the SvgPath folder
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem)
IF (BOOST_FOUND)
    INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})
ENDIF()

    
INCLUDE_DIRECTORIES (src src/svg_path src/route_planner)

SET (LIB_SRCS 
    src/DebugLogger.cpp 
    src/GlobalClock.cpp 
    src/Threaded.cpp 
    src/svg_path/Station.cpp 
    src/svg_path/StationDoesNotExist.cpp 
	src/svg_path/StationList.cpp 
	src/svg_path/StationPath.cpp 
    src/svg_path/SvgPath.cpp 
	src/route_planner/RoutePlanner.cpp 
	src/route_planner/MissionComm.cpp 
	src/route_planner/HTTPClient.cpp 
	src/route_planner/DBInterface.cpp 
	src/route_planner/SimulatedRoutePlanner.cpp 
	src/route_planner/PassengerComm.cpp
)

ADD_LIBRARY (bookingcommon SHARED ${LIB_SRCS})
TARGET_LINK_LIBRARIES(bookingcommon ${TINYXML_LIBRARIES} ${CURL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY})

ADD_EXECUTABLE(test_DBInterface src/route_planner/test_DBInterface.cpp)
TARGET_LINK_LIBRARIES(test_DBInterface bookingcommon)

ADD_EXECUTABLE(test_SvgPath src/svg_path/test_SvgPath.cpp)
TARGET_LINK_LIBRARIES(test_SvgPath bookingcommon)
