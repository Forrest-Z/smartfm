// Definition of the Socket class

#ifndef SOCKET_H
#define SOCKET_H


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <arpa/inet.h>


const int MAXHOSTNAME = 200;
const int MAXCONNECTIONS = 5;
const int MAXRECV = 500;
const int MSGNOSIGNAL = 0; // defined by dgame

class Socket
{
 public:
  Socket();
  virtual ~Socket();

  // Server initialization
  bool create();
  bool bind ( const int port );
  bool listen() const;
  bool accept ( Socket& ) const;

  // Client initialization
  bool connect ( const std::string host, const int port );

  // Data Transimission
  bool send ( const std::string ) const;
  int recv ( std::string& ) const;

  // Set socket options
  bool setSocketBlocking ( const bool );
  bool setReuseAddr(const int reuse);

  // Check whether this socket is valid
  bool isValid() const;

  // Close connection
  bool close() const;

 private:

  int m_sock;
  sockaddr_in m_addr;
  bool m_blocking;

};


#endif // SOCKET_H
