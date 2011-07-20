#include "ServerSocket.hh"
#include "SocketException.hh"
#include <iostream>
#include <sstream>

ServerSocket::ServerSocket(int port)
{
  this->init(port);
}

void ServerSocket::init(int port)
{
  if (!Socket::create())
    throw SocketException ( SOCKET_CREATE_ERROR, "Socket cannot be created." );
  if (!Socket::bind ( port )) {
    std::ostringstream ss;
    ss << "Cannot bind to " << port;
    throw SocketException ( SOCKET_BIND_ERROR, ss.str() );
  }
  if (!Socket::listen())
    throw SocketException ( SOCKET_LISTEN_ERROR, "Socket cannot be listening." );    
}

const ServerSocket& ServerSocket::operator << ( const std::string& s ) const
{
  if ( !Socket::send ( s ) )
    throw SocketException ( SOCKET_SEND_ERROR, "Cannot send message to to socket." );
  
  return *this;
}


const ServerSocket& ServerSocket::operator >> ( std::string& s ) const
{
  if ( ! Socket::recv ( s ) )
    throw SocketException ( SOCKET_RECV_ERROR, "Cannot receive message from socket." );

  return *this;
}

void ServerSocket::accept ( ServerSocket& svrsocket)
{
  if (!Socket::accept(svrsocket))
    throw SocketException ( SOCKET_ACCEPT_ERROR, "Cannot accept communication." );
}

bool ServerSocket::setSocketBlocking(const bool blocking)
{
  return Socket::setSocketBlocking(blocking);
}
