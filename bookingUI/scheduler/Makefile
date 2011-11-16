SOCKET_ROOT_DIR = ../socket_handler
SOCKET_LIB = $(SOCKET_ROOT_DIR)/libsocket.a
RP_ROOT_DIR := $(shell rospack find golfcar_route_planner)

# Tool definitions
CC = gcc
CPP = g++
INCLUDE = -I$(SOCKET_ROOT_DIR) -I$(RP_ROOT_DIR)/src
CFLAGS += -g $(INCLUDE)
#CFLAGS +=-g -Wall $(INCLUDE)
CPPFLAGS += $(CFLAGS)
LDFLAGS += -lcurses -lncurses -lpthread

all: scheduler dummyrp

scheduler: SchedulerMain.o SchedulerUI.o SchedulerTalker.o VehicleTalker.o Scheduler.o StationNetwork.o $(SOCKET_LIB)
	$(CPP) $^ $(LDFLAGS) -o $@

dummyrp: RoutePlanner.o DummyRoutePlannerMain.o $(SOCKET_LIB)
	$(CPP) $^ $(LDFLAGS) -o $@

RoutePlanner.o: $(RP_ROOT_DIR)/src/RoutePlanner.cc
	$(CPP) -c $(CPPFLAGS) $^ -o $@

dep:
	makedepend -Y -- $(CPPFLAGS) -- $(wildcard *.cc) 2>/dev/null

clean:
	rm -f *.o scheduler dummyrp *~ \#*
