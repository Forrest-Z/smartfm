#ifndef __DB_INTERFACE_EXCEPTION_H__
#define __DB_INTERFACE_EXCEPTION_H__

#include <sstream>
#include <exception>

class DBInterfaceException : public std::exception
{
protected:
    std::string msg;
public:
    DBInterfaceException(std::string s="") throw() : std::exception(), msg(s) { }
    virtual ~DBInterfaceException() throw() { }
    virtual const char * what() throw() { return msg.c_str(); }
};

class HTTPClientException : public DBInterfaceException
{
public:
    HTTPClientException(std::string msg) throw()
        : DBInterfaceException(std::string("HTTPClientException: ")+msg) { }
    virtual ~HTTPClientException() throw() { }
};

class VehicleNotFoundException : public DBInterfaceException
{
public:
    VehicleNotFoundException(std::string vid) throw()
        : DBInterfaceException(std::string("VehicleNotFoundException: ")+vid) { }
    virtual ~VehicleNotFoundException() throw() { }
};

class TaskNotFoundException : public DBInterfaceException
{
public:
    TaskNotFoundException(unsigned id) throw() : DBInterfaceException("")
    {
        std::stringstream ss;
        ss <<"TaskNotFoundException: " <<id;
        this->msg = ss.str();
    }

    virtual ~TaskNotFoundException() throw() { }
};

#endif
