#include <string.h>
#include <iostream>
#include <errno.h>
#include <fcntl.h>

#include "socket_handler.hh"


#define MAXRECV 200

Socket::Socket()
{
    memset ( &m_addr, 0, sizeof ( m_addr ) );
    m_sock = -1;
    m_blocking = true;
}


Socket::~Socket()
{
    if ( isValid() )
        ::close ( m_sock );
}


void Socket::create()
{
    m_sock = socket ( AF_INET, SOCK_STREAM, 0 );
    if ( !isValid() )
        throw SocketException(SOCKET_CREATE_ERROR, "Socket cannot be created.");
}


void Socket::send ( const std::string & s ) const
{
    if( !isValid() )
        throw SocketException( SOCKET_INVALID_ERROR, "Invalid Socket.");

    int status = ::send ( m_sock, s.c_str(), s.size(), MSG_NOSIGNAL );
    if ( status == -1 )
        throw SocketException ( SOCKET_SEND_ERROR, "Cannot send message to to socket." );
}


void Socket::recv(std::string& s) const
{
    if( !isValid() )
        throw SocketException( SOCKET_INVALID_ERROR, "Invalid Socket.");

    char buf [ MAXRECV + 1 ];
    memset ( buf, 0, MAXRECV + 1 );

    if ( ::recv ( m_sock, buf, MAXRECV, 0 ) <= 0 )
        throw SocketException ( SOCKET_RECV_ERROR, "Cannot receive message from socket." );
    else
        s = buf;
}


std::string Socket::recv() const
{
    std::string s;
    recv(s);
    return s;
}


const Socket& Socket::operator << ( const std::string& s ) const
{
    Socket::send(s);
    return *this;
}


const Socket& Socket::operator >> ( std::string& s ) const
{
    Socket::recv(s);
    return *this;
}


void Socket::setSocketBlocking ( bool blocking )
{
    if( !isValid() )
        throw SocketException( SOCKET_INVALID_ERROR, "Invalid Socket.");

    if (blocking == m_blocking)
        return;

    /*
     *  if (ioctl(m_sock,FIONBIO,(char *)&blocking) != -1) {
     *    m_blocking = blocking;
     *    return true;
     }
     return false;
     */

    int opts = fcntl ( m_sock, F_GETFL );
    if ( opts < 0 )
        throw SocketException(SOCKET_BLOCKING_ERROR, "Could not set blocking mode");

    opts |= blocking ? ~O_NONBLOCK : O_NONBLOCK;

    if (fcntl( m_sock, F_SETFL,opts ) == -1)
        throw SocketException(SOCKET_BLOCKING_ERROR, "Could not set blocking mode");

    m_blocking = blocking;
}


void Socket::setReuseAddr(const int reuse)
{
    if( !isValid() )
        throw SocketException( SOCKET_INVALID_ERROR, "Invalid Socket.");
    if ( setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &reuse, sizeof (reuse) ) == -1 )
        throw SocketException( SOCKET_REUSE_ERROR, "Could not set to reuse addr.");
}


bool Socket::isValid() const
{
    return m_sock != -1;
}


void Socket::close() const
{
    if( !isValid() )
        throw SocketException( SOCKET_INVALID_ERROR, "Invalid Socket.");
    ::close ( m_sock );
}


void Socket::bind ( int port )
{
    if ( ! isValid() )
        throw SocketException( SOCKET_INVALID_ERROR, "Invalid Socket.");

    m_addr.sin_family = AF_INET;
    m_addr.sin_addr.s_addr = INADDR_ANY;
    m_addr.sin_port = htons ( port );

    int s = ::bind ( m_sock, ( struct sockaddr * ) &m_addr, sizeof ( m_addr ) );
    if ( s == -1 )
        throw SocketException(SOCKET_BIND_ERROR, "Could not bind socket.");
}


void Socket::listen() const
{
    if ( ! isValid() )
        throw SocketException( SOCKET_INVALID_ERROR, "Invalid Socket.");

    int MAXCONNECTIONS = 10;
    int s = ::listen ( m_sock, 10 );
    if ( s == -1 )
        throw SocketException(SOCKET_LISTEN_ERROR, "Could not listen on socket.");
}


void Socket::accept ( Socket& new_socket ) const
{
    if ( ! isValid() )
        throw SocketException( SOCKET_INVALID_ERROR, "Invalid Socket.");

    int l = sizeof ( m_addr );
    new_socket.m_sock = ::accept ( m_sock, (sockaddr *) &m_addr, (socklen_t *) &l );

    if ( new_socket.m_sock <= 0 )
        throw SocketException( SOCKET_ACCEPT_ERROR, "Could not accept on socket.");
}
