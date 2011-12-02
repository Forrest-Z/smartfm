#include <errno.h>
#include <iostream>
#include <sstream>
#include "socket_handler.h"

void ClientSocket::connect ( const std::string & host, int port )
{
    Socket::create();

    std::ostringstream ss;
    ss << "Cannot connect to " << host << ":" << port;
    SocketException e( SOCKET_CONNECT_ERROR, ss.str() );

    m_addr.sin_family = AF_INET;
    m_addr.sin_port = htons ( port );

    int status = inet_pton ( AF_INET, host.c_str(), &m_addr.sin_addr );
    if ( errno == EAFNOSUPPORT ) throw e;

    status = ::connect ( m_sock, ( sockaddr * ) &m_addr, sizeof ( m_addr ) );
    if ( status != 0 ) throw e;
}
