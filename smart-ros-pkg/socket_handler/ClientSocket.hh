// Definition of the ClientSocket class

#ifndef ClientSocket_class
#define ClientSocket_class

#include "Socket.hh"


class ClientSocket : private Socket
{
 public:
  ClientSocket();
  ClientSocket( std::string host, int port );
  virtual ~ClientSocket(){Socket::close();};

  void init(std::string host, int port);
  const ClientSocket& operator << ( const std::string& ) const;
  const ClientSocket& operator >> ( std::string& ) const;
  bool setSocketBlocking ( const bool );
};


#endif
