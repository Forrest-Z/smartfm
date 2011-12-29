#include <iostream>
#include <stdexcept>

#include "SvgPath.h"

using namespace std;

int main(int argc, char* argv[])
{
    try
    {
        SvgPath sp;
        sp.loadFile(argv[1], 0.1);
        StationPath pose= sp.getPath("DCC_EA");
        StationPath pose2= sp.getPath("EA_DCC");
        cout <<"Size DCC_MCD=" <<pose.size() <<' ' <<" Size MCD_DCC=" <<pose2.size() <<endl;
        vector<SlowZone> sz = sp.getSlowZone();
        cout <<sz[0].x_ <<' ' <<sz[0].y_ <<' ' <<sz[0].r_ <<endl;
    }
    catch (runtime_error & error)
    {
        cout <<"Error: " <<error.what() <<endl;
    }
}
