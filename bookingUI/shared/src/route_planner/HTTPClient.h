#ifndef __HTTP_CLIENT_H__
#define __HTTP_CLIENT_H__

#include <string>
#include <sstream>
#include <iomanip>

/// A HTTP client
class HTTPClient
{
private:
    std::string url;
    std::string urlEncodedParameters;

public:
    /// Creates a client that will communicate with the script on the host.
    HTTPClient(std::string baseurl, std::string scriptname);

    /// Closes the connection and release ressources.
    ~HTTPClient();

    /// Add a (name,value) parameter to the call (GET)
    template<class T>
    HTTPClient & addParam(std::string name, T value)
    {
        std::stringstream ss;
        if( urlEncodedParameters.length()>0 )
            ss <<'&';
        ss <<name <<'=' <<std::setprecision(12) <<value;
        urlEncodedParameters = urlEncodedParameters + ss.str();
        return *this;
    }


    /// Connect to the server and retrieve the data.
    std::string connect();
};

#endif
