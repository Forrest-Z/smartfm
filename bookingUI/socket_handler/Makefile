# Tool definitions
CC = gcc
CPP= g++

all: libsocket.a

libsocket.a: ClientSocket.o ServerSocket.o Socket.o
	ar r $@ $^

clean:
	rm -f *.o *.a *~