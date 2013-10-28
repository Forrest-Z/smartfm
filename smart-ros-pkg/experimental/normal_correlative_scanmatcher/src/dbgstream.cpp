/*
** file: dbgstream.cpp -- debugging stream
*/
#include "dbgstream.h"
#include <fstream>
#include <iostream>
using namespace std;

dbgbuf::~dbgbuf()
{
    flushMsg();
}

void dbgbuf::flushMsg()
{
	if (msg.length() > 0) {
		std::cout<<msg<<endl;

		msg.erase();	// erase message buffer
	}
}


int dbgbuf::overflow(int c)
{
	if(enable_output_)
	{
		if (c == '\n') {
			flushMsg();
		} else {
			msg += c;
		}
		return c == -1 ? -1 : ' ';
	}
    else return -1;
}
/*
int main(int argc, char** argcv)
{
	dbgstream dbg;
	double number = 1.234;
	dbg << "Hello, World."<<number << endl;


	return 0;
}*/
