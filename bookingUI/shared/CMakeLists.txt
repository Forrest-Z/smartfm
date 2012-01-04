cmake_minimum_required (VERSION 2.8)
project (BookingCommon)

SET (CMAKE_CXX_FLAGS "-g -Wall")

# TinyXml is required. Since there was an API change at 2.6 we need to check
# the version number and define a flag accordingly
include(FindPkgConfig)
pkg_check_modules(TINYXML REQUIRED tinyxml)
pkg_check_modules(TINYXML26 tinyxml>=2.6) #QUIET only works with cmake>=2.8.2
if( NOT TINYXML26_FOUND )
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTINYXML_API_PRE26")
    MESSAGE("Using old tinyxml API (pre 2.6). Setting the compilation flag TINYXML_API_PRE26.")
endif( NOT TINYXML26_FOUND )
include_directories(${TINYXML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIRS})

# We also need libcurl
find_package(CURL REQUIRED)
include_directories(${CURL_INCLUDE_DIRS})
link_directories(${CURL_LIBRARY_DIRS})

# boost/filesystem required to find the SvgPath folder
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem)
IF (BOOST_FOUND)
    INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})
ENDIF()

    
INCLUDE_DIRECTORIES (src)

SET (LIB_SRCS src/DebugLogger.cpp src/Station.cpp src/StationDoesNotExist.cpp 
		src/StationList.cpp src/StationPath.cpp src/Threaded.cpp 
		src/RoutePlanner.cpp src/MissionComm.cpp src/HTTPClient.cpp 
		src/DBInterface.cpp src/SvgPath.cpp src/GlobalClock.cpp 
		src/SimulatedRoutePlanner.cpp src/PassengerComm.cpp)

ADD_LIBRARY (bookingcommon SHARED ${LIB_SRCS})
target_link_libraries(bookingcommon ${TINYXML_LIBRARIES} ${CURL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY})

ADD_EXECUTABLE(test_DBInterface src/test_DBInterface.cpp)
target_link_libraries(test_DBInterface bookingcommon)

ADD_EXECUTABLE(test_SvgPath src/test_SvgPath.cpp)
target_link_libraries(test_SvgPath bookingcommon)
