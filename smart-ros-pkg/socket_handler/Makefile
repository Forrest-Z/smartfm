# Tool definitions
CC = gcc
CPP= g++

all: socket.a

socket.a: ClientSocket.o Socket.o SocketException.o
	ar r $@ $^

clean: 
	rm -f *.o *.a *~