#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h> //readlink

#include <iostream>
#include <exception>
#include <sstream>

#include <boost/filesystem.hpp>

#include <wordexp.h>

using namespace std;



const char *filename = "~/SvgPath/ALL_1215.svg";
stringstream errMsgStream;


unsigned ntests = 0, nsuccess = 0;

void dotest(bool (*test)(void), const char *testname)
{
	ntests ++;
    try {
    	errMsgStream.str("");
        if( test() ) {
        	nsuccess ++;
            cout <<"SUCCESS: " <<testname <<endl;
        }
        else {
            cout <<"FAILED : " <<testname <<". " <<errMsgStream.str() <<endl;
        }
    } catch( exception & e ) {
        cout <<"FAILED : " <<testname <<":" <<endl <<"\t" <<e.what() <<endl;
    }
}

#define TEST(testname) dotest(testname,#testname)


bool testFopen()
{
	FILE *f = fopen(filename,"r");
	if( f==NULL ) {
		errMsgStream <<"Could not open file " <<filename <<". " <<strerror(errno) <<".";
		return false;
	}
	fclose(f);
	return true;
}

bool testReadLink()
{
	char buf[1024];
	ssize_t s = readlink(filename, buf, 1024);
	if( s<0 ) {
		errMsgStream <<"Error in readlink. " <<strerror(errno) <<".";
		return false;
	}
	if( s==1024 ) {
		errMsgStream <<"Error in readlink (buf too small)." <<endl;
		return false;
	}
	buf[s] = 0;
	FILE *f = fopen(buf,"r");
	if( f==NULL ) {
		errMsgStream <<"Could not open file " <<buf <<". " <<strerror(errno) <<".";
		return false;
	}
	fclose(f);
	return true;
}

bool testBoost()
{
	return boost::filesystem::exists(filename);
}

bool testWordExp()
{
	wordexp_t p;
	wordexp(filename, &p, 0);
	char **w = p.we_wordv;
	if( p.we_wordc==0 ) {
		errMsgStream <<"wordexp-ed to nothing." <<endl;
		return false;
	}
	if( p.we_wordc>1 ) {
		errMsgStream <<"wordexp-ed to many results: ";
		for (unsigned i = 0; i < p.we_wordc; i++) errMsgStream <<w[i] <<", ";
		errMsgStream <<endl;
		return false;
	}
	string res = w[0];
	wordfree(&p);
	return boost::filesystem::exists(res);
}

int main()
{
	TEST(testFopen);
	TEST(testReadLink);
	TEST(testBoost);
	TEST(testWordExp);
	return 0;
}


