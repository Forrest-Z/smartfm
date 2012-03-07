LIBS    = -lopencv_core -lopencv_highgui
INCS    = -I/usr/include/opencv-2.3.1
CFLAGS  = -Wall -g


all: grabcam multicam multicamclose

grabcam: grabcam.cpp
	gcc $(CFLAGS) $(INCS) $< -o $@ $(LIBS)

multicam: multicam.cpp
	gcc $(CFLAGS) $(INCS) $< -o $@ $(LIBS)

multicamclose: multicamclose.cpp
	gcc $(CFLAGS) $(INCS) $< -o $@ $(LIBS)