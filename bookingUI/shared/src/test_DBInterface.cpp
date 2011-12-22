#include <stdio.h>

#include <iostream>
#include <string>

#include "DBInterface.h"

using namespace std;

unsigned ntests = 0, nsuccess = 0;

void dotest(bool (*test)(void), const char *testname)
{
	ntests ++;
    try {
        if( test() ) {
        	nsuccess ++;
            cout <<"SUCCESS: ";
        }
        else {
            cout <<"FAILED : ";
        }
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
    try {
        DBInterface::Vehicle v = dbi->getVehicleEntry();
    } catch( VehicleNotFoundException & e ) {
        return true;
    }
    return false;
}

bool testSetCurrentLocation()
{
    dbi->setCurrentLocation("DUMMY");
    DBInterface::Vehicle v = dbi->getVehicleEntry();
    return strcasecmp(v.currentLocation.c_str(), "DUMMY")==0;
}

bool testSetVehicleStatus()
{
    dbi->setVehicleStatus("NotAvailable");
    DBInterface::Vehicle v = dbi->getVehicleEntry();
    return strcasecmp(v.status.c_str(), "NotAvailable")==0;
}

bool testSetGeoLocation()
{
    dbi->setGeoLocation(1, 103);
    DBInterface::Vehicle v = dbi->getVehicleEntry();
    return v.latitude==1 && v.longitude==103;
}

bool testSetETA()
{
    dbi->setETA(10);
    DBInterface::Vehicle v = dbi->getVehicleEntry();
    return v.eta==10;
}


int main()
{
    TEST(testCreateInterface);
    TEST(testIdentify);
    
    TEST(testSetCurrentLocation);
    TEST(testSetVehicleStatus);
    TEST(testSetGeoLocation);
    TEST(testSetETA);
    
    TEST(testDeleteVehicle);

    cout <<"DONE. PASSED " <<nsuccess <<" TESTS OUT OF " <<ntests <<"." <<endl;

    if(dbi) delete dbi;
    return 0;
}
