#include <string>
#include <sstream>
#include <exception>

#include "StationPath.h"

using std::string;
using std::exception;
using std::stringstream;


StationDoesNotExistException::StationDoesNotExistException() throw()
: exception()
{

}

StationDoesNotExistException::StationDoesNotExistException(const string & s) throw()
: exception(), msg(s)
{

}

StationDoesNotExistException::StationDoesNotExistException(unsigned i) throw()
: exception()
{
    stringstream s;
    s <<i;
    msg = s.str();
}

StationDoesNotExistException::~StationDoesNotExistException() throw()
{

}

const char* StationDoesNotExistException::what() const throw()
{
    return msg.c_str();
}