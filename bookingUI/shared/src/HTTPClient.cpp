#include <stdlib.h>
#include <string.h>

#include <string>
#include <stdexcept>

// libcurl is used for the heavy lifting.
#include <curl/curl.h>

#include "DBInterfaceException.h"
#include "HTTPClient.h"

using namespace std;

/*
std::string url;
std::string urlEncodedParameters;
*/

HTTPClient::HTTPClient(string baseurl, string scriptname)
{
    this->url = baseurl + "/" + scriptname;
}

HTTPClient::~HTTPClient()
{

}


// See http://curl.haxx.se/libcurl/c/getinmemory.html
// for the source of this code

struct MemoryStruct {
    char *memory;
    size_t size;
};

static size_t
WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    size_t realsize = size * nmemb;
    struct MemoryStruct *mem = (struct MemoryStruct *)userp;

    mem->memory = (char *)realloc(mem->memory, mem->size + realsize + 1);
    if (mem->memory == NULL)
        throw HTTPClientException("not enough memory (realloc returned NULL)");

    memcpy(&(mem->memory[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;

    return realsize;
}

std::string HTTPClient::connect()
{
    CURL *curl = curl_easy_init();
    if( curl==0 ) throw HTTPClientException("Failed to initialize a CURL handle");

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, urlEncodedParameters.c_str());

    struct MemoryStruct chunk;
    chunk.memory = (char *)malloc(1);  /* will be grown as needed by the realloc above */
    if( chunk.memory==NULL )
        throw HTTPClientException("Failed to allocate chunk.memory");
    chunk.size = 0;    /* no data at this point */
    chunk.memory[0] = 0;

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);
    CURLcode res = curl_easy_perform(curl);
    if( res!=0 ) throw HTTPClientException("curl_easy_perform Failed");

    string data;
    if(chunk.memory) {
        data = chunk.memory;
        free(chunk.memory);
    }

    /* we're done with libcurl, so clean it up */
    curl_easy_cleanup(curl);
    curl_global_cleanup();

    return data;
}
