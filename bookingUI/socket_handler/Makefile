# Tool definitions
CC = gcc
CPP= g++

all: socket.a

socket.a: ClientSocket.o ServerSocket.o Socket.o
	ar r $@ $^

clean:
	rm -f *.o *.a *~