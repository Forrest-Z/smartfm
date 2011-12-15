#include <stdio.h>

#include <iostream>
#include <string>

#include "DBInterface.h"

using namespace std;

void dotest(bool (*test)(void), const char *testname)
{
    try {
        if( test() )
            cout <<"SUCCESS: ";
        else
            cout <<"FAILED : ";
        cout <<testname <<endl;
    } catch( DBInterfaceException & e ) {
        cout <<"FAILED : " <<testname <<":" <<endl <<"\t" <<e.what() <<endl;
    } catch( exception & e ) {
        cout <<"FAILED : " <<testname <<":" <<endl <<"\t" <<e.what() <<endl;
    }
}

#define TEST(testname) dotest(testname,#testname)

DBInterface *dbi=0;
string vehicleID = "golfcart1";

bool testCreateInterface()
{
    dbi = new DBInterface("http://fmautonomy.no-ip.info/dbserver", vehicleID);
    return dbi!=0;
}

bool testIdentify()
{
    dbi->identify();
    DBInterface::Vehicle v = dbi->getVehicleEntry();
    return strcasecmp(v.vehicleID.c_str(), vehicleID.c_str())==0;
}

bool testDeleteVehicle()
{
    dbi->deleteVehicle();
    return true;
}


int main()
{
    TEST(testCreateInterface);
    TEST(testIdentify);

    TEST(testDeleteVehicle);
    if(dbi) delete dbi;
    return 0;
}
