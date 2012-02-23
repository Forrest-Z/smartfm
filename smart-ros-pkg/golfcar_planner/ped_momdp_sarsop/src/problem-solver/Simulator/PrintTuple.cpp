#include "PrintTuple.h"

using namespace std;

namespace momdp
{

void printTuple(map<string, string> tuple, ofstream & streamOut)
{
    streamOut << "(";
    map<string, string>::iterator iter = tuple.begin();
    for( ; iter != tuple.end() ; )
    {
        streamOut << iter->second;
        if( ++iter != tuple.end() )
            streamOut << ",";
    }
    streamOut << ")" << endl;
}

}