#ifndef SOCKET_EXCEPTION_H
#define SOCKET_EXCEPTION_H

#include <string>


enum SocketError
  {
    SOCKET_NO_ERROR=0,
    SOCKET_CREATE_ERROR,
    SOCKET_BIND_ERROR,
    SOCKET_LISTEN_ERROR,
    SOCKET_ACCEPT_ERROR,
    SOCKET_CONNECT_ERROR,
    SOCKET_SEND_ERROR,
    SOCKET_RECV_ERROR
  };

class SocketException
{
 public:
  SocketException(SocketError, const std::string& );
  ~SocketException() {};
  int getErrCode()    { return errorCode; }
  std::string& getErrMsg() { return errorMsg; }

 private:
  SocketError errorCode;
  std::string errorMsg;
};

#endif // SOCKET_EXCEPTION_H
