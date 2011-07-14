############################################################
# 
# This file should only contain CFLAGS_XXX and LDFLAGS_XXX directives.
# CFLAGS and LDFLAGS themselves should NOT be set: that is the job
# for the actual Makefiles (which will combine the flags given here)
#
# *** DO NOT SET CFLAGS or LDFLAGS  ***
#
# Our recommended flags for all projects.
# -Wno-format-zero-length: permit printf("");
# -Wno-unused-parameter: permit a function to ignore an argument


CFLAGS_STD   := -g -std=gnu99 \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \
	-Wall -Wno-unused-parameter 
CXXFLAGS_STD := -g \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \
	-Wall -Wno-unused-parameter -Wno-sign-compare -D__STDC_FORMAT_MACROS
LDFLAGS_STD  := -lm

ROOT_PATH    = $(shell pwd)
BIN_PATH     = $(ROOT_PATH)

CC           := gcc
LD           := gcc
CXX          := g++
LDXX         := g++


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
