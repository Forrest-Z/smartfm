include common.mk

CFLAGS = $(CFLAGS_STD) 

CXXFLAGS = $(CXXFLAGS_STD) 

LDFLAGS = $(LDFLAGS_STD) -llapack -lblas -g


RRTS_OBJS = 	rrts_main_dubins.o \
		kdtree.o


RRTS=$(BIN_PATH)/ex-rrts
RRTS_DEPENDENCIES:=$(RRTS_OBJS) $(LDFLAGS_COMMON)


all: $(RRTS) # $(OPTSYSTEM_TEST) 


$(RRTS): $(RRTS_DEPENDENCIES)
	$(CXX) -g -o $@ $(RRTS_OBJS) $(LDFLAGS) $(CXXFLAGS)

clean:
	rm -f *.o *.a *~ $(OPTTREE) $(OPTSYSTEM_TEST) $(RRTS)
