INCLUDE_PATH = ../socket_handler
SOCKET_DIR = ../socket_handler

# Tool definitions
CC = gcc
CPP= g++
INCLUDE=-I$(INCLUDE_PATH)
CFLAGS += -g $(INCLUDE)
#CFLAGS += -g -Wall $(INCLUDE)
CPPFLAGS+= -pthread $(CFLAGS)
LDFLAGS+=-lcurses -lncurses
SRCS=StationNetwork.cc Scheduler.cc SchedulerUI.cc SchedulerTalker.cc VehicleTalker.cc SchedulerMain.cc
EXECNAME=scheduler

all: $(EXECNAME) dummyrp

$(EXECNAME): SchedulerMain.o SchedulerUI.o SchedulerTalker.o VehicleTalker.o Scheduler.o StationNetwork.o $(SOCKET_DIR)/socket.a
	$(CPP) $(CPPFLAGS) $^ $(LDFLAGS) -o $@

dummyrp: RoutePlanner.o DummyRoutePlannerMain.o $(SOCKET_DIR)/socket.a
	$(CPP) $(CPPFLAGS) $^ $(LDFLAGS) -o $@

dep:
	makedepend -Y -- $(CPPFLAGS) $(CFLAGS) -- $(SRCS) 2>/dev/null

clean:
	rm -f *.o $(EXECNAME) dummyrp *~ \#*
