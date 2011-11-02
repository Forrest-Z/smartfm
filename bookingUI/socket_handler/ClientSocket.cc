// Implementation of the ClientSocket class

#include "ClientSocket.hh"
#include "SocketException.hh"
#include <iostream>
#include <sstream>

ClientSocket::ClientSocket()
{
}

ClientSocket::ClientSocket ( std::string host, int port )
{
  this->init(host, port);
}

void ClientSocket::init ( std::string host, int port )
{
  if ( !Socket::create() )
    {
      throw SocketException ( SOCKET_CREATE_ERROR, "Socket cannot be created." );
    }

  if ( !Socket::connect ( host, port ) )
    {
      std::ostringstream ss;
      ss << "Cannot connect to " << host << ":" << port;
      throw SocketException ( SOCKET_CONNECT_ERROR, ss.str() );
    }
}

const ClientSocket& ClientSocket::operator << ( const std::string& s ) const
{
  //Socket::send(s);
  
  if ( !Socket::send ( s ) )
    {
      throw SocketException ( SOCKET_SEND_ERROR, "Cannot send message to to socket." );
    }
  
  return *this;

}


const ClientSocket& ClientSocket::operator >> ( std::string& s ) const
{
  //Socket::recv(s);
  
  if ( ! Socket::recv ( s ) )
    {
      throw SocketException ( SOCKET_RECV_ERROR, "Cannot receive message from socket." );
    }
  
  return *this;
}


bool ClientSocket::setSocketBlocking(const bool blocking)
{
  return Socket::setSocketBlocking(blocking);
}
