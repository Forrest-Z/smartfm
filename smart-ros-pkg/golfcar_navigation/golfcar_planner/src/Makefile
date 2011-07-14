CC           := gcc
LD           := gcc
CXX          := g++
LDXX         := g++


CXXFLAGS_STD := -g \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \

CFLAGS_STD   := -g -std=gnu99 \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \

LDFLAGS_STD  := -lm

ROOT_PATH    = $(shell pwd)
BIN_PATH     = $(ROOT_PATH)

%.o: %.c %.h
	@echo "    [$@]"
	$(CC) $(CFLAGS) -c $< 

%.o: %.c
	@echo "    [$@]"
	$(CC) $(CFLAGS) -c $< 

%.o: %.cpp %.h
	@echo "    [$@]"
	$(CXX) -c -o $@ $< $(CXXFLAGS)

%.o: %.cpp
	@echo "    [$@]"
	$(CXX) -c -o $@ $< $(CXXFLAGS)
CFLAGS = $(CFLAGS_STD) 

CXXFLAGS = $(CXXFLAGS_STD) 

LDFLAGS = $(LDFLAGS_STD) -g


RRTS_OBJS = golfcar_planner.o \
		kdtree.o


RRTS=$(BIN_PATH)/ex-rrts
RRTS_DEPENDENCIES:=$(RRTS_OBJS) $(LDFLAGS_COMMON)


all: $(RRTS) # $(OPTSYSTEM_TEST) 


$(RRTS): $(RRTS_DEPENDENCIES)
	$(CXX) -g -o $@ $(RRTS_OBJS) $(LDFLAGS) $(CXXFLAGS)

clean:
	rm -f *.o *.a *~ $(OPTTREE) $(OPTSYSTEM_TEST) $(RRTS)
