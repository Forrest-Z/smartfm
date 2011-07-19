#include "SocketException.hh"

using namespace std;

SocketException::SocketException(SocketError errorCode, const std::string& errorMsg) {
    this->errorCode = errorCode;
    this->errorMsg = errorMsg;
  }
