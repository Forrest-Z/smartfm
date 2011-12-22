#ifndef __SOCKET_HANDLER_HH__
#define __SOCKET_HANDLER_HH__

#include <string>
#include <exception>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

enum SocketError
{
    SOCKET_NO_ERROR=0,
    SOCKET_CREATE_ERROR,
    SOCKET_BIND_ERROR,
    SOCKET_LISTEN_ERROR,
    SOCKET_ACCEPT_ERROR,
    SOCKET_CONNECT_ERROR,
    SOCKET_SEND_ERROR,
    SOCKET_RECV_ERROR,
    SOCKET_INVALID_ERROR,
    SOCKET_BLOCKING_ERROR,
    SOCKET_REUSE_ERROR
};

class SocketException : public std::exception
{
public:
    SocketException(SocketError e, const std::string & s) throw()
    : std::exception(), errorCode(e), errorMsg(s)
    {

    }

    virtual ~SocketException() throw() { }

    int getErrCode() const throw() { return errorCode; }
    const std::string & getErrMsg() const throw() { return errorMsg; }
    virtual const char* what() const throw() {return errorMsg.c_str();}

private:
    SocketError errorCode;
    std::string errorMsg;
};

/// A Base class for sockets. Implements the messaging part.
class Socket
{
public:
    Socket();
    virtual ~Socket();

    void send ( const std::string & ) const;
    void recv ( std::string & ) const;
    std::string recv () const;
    const Socket& operator << ( const std::string& ) const;
    const Socket& operator >> ( std::string& ) const;

    void setSocketBlocking ( bool );

    void setReuseAddr(int reuse);

    /// Check whether this socket is valid
    bool isValid() const;

    /// Close connection
    void close() const;

    void bind ( int port );
    void listen() const;
    void accept ( Socket& ) const;

protected:
    void create();
    int m_sock;
    sockaddr_in m_addr;
    bool m_blocking;
};


class ClientSocket : public Socket
{
public:
    ClientSocket() : Socket() { }
    ClientSocket(const std::string & host, int port) : Socket()
    {
        this->connect(host, port);
    }
    void connect(const std::string & host, int port );
};

class ServerSocket : protected Socket
{
public:
    ServerSocket() : Socket() {};
    ServerSocket(int port) : Socket() { init(port); }

    Socket accept() const;
    void accept(Socket &) const;

    void init(int port);
};


#endif
