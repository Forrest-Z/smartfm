cmake_minimum_required (VERSION 2.8)
project (car_detect)

SET (CMAKE_CXX_FLAGS "-g -Wall")

# OpenCV
FIND_PACKAGE( OpenCV REQUIRED )

SET( SRCS 	
	src/GlobalClock.cpp
	src/MovingAvgBackground.cpp
	src/MovingObjectDetector.cpp
	src/Blob.cpp
	src/BlobExtractor.cpp
	src/BlobFilter.cpp 
	src/BlobTracker.cpp
	src/CarDetector.cpp
)
			
ADD_LIBRARY(cardetect ${SRCS})

ADD_EXECUTABLE(test_background src/test_background.cpp )
target_link_libraries(test_background cardetect ${OpenCV_LIBS})

ADD_EXECUTABLE(test_motion_detect src/test_motion_detect.cpp   )
target_link_libraries(test_motion_detect cardetect ${OpenCV_LIBS})

ADD_EXECUTABLE(test_detection src/test_detection.cpp)
target_link_libraries(test_detection cardetect ${OpenCV_LIBS})

ADD_EXECUTABLE(test_playback_time src/test_playback_time.cpp)
target_link_libraries(test_playback_time cardetect ${OpenCV_LIBS})

ADD_EXECUTABLE(multi_cam_detect src/multi_cam_detect.cpp)
target_link_libraries(multi_cam_detect cardetect ${OpenCV_LIBS})