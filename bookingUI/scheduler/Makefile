INCLUDE_PATH = ../socket_handler
SOCKET_LIB = ../socket_handler/socket.a

# Tool definitions
CC =gcc
CPP=g++
INCLUDE=-I$(INCLUDE_PATH)
CFLAGS +=-g $(INCLUDE)
#CFLAGS +=-g -Wall $(INCLUDE)
CPPFLAGS+=$(CFLAGS)
LDFLAGS+=-lcurses -lncurses -lpthread
SRCS=StationNetwork.cc Scheduler.cc SchedulerUI.cc SchedulerTalker.cc VehicleTalker.cc SchedulerMain.cc

all: scheduler dummyrp

scheduler: SchedulerMain.o SchedulerUI.o SchedulerTalker.o VehicleTalker.o Scheduler.o StationNetwork.o $(SOCKET_LIB)
	$(CPP) $^ $(LDFLAGS) -o $@

dummyrp: RoutePlanner.o DummyRoutePlannerMain.o $(SOCKET_LIB)
	$(CPP) $^ $(LDFLAGS) -o $@

dep:
	makedepend -Y -- $(CPPFLAGS) -- $(SRCS) 2>/dev/null

clean:
	rm -f *.o scheduler dummyrp *~ \#*
