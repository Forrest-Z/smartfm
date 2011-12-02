#include <iostream>
#include <sstream>
#include "socket_handler.h"

void ServerSocket::init(int port)
{
    create();
    bind(port);
    listen();
}

void ServerSocket::accept ( Socket& s) const
{
    accept(s);
}

Socket ServerSocket::accept() const
{
    Socket s;
    accept(s);
    return s;
}
