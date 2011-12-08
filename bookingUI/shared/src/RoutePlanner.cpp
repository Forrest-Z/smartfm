#include <stdlib.h>

#include <iostream>

#include "RoutePlanner.h"

using namespace std;



RoutePlanner::RoutePlanner(StationPaths & sp)
    : sp_(sp), state_(sUninit)
{

}

void RoutePlanner::setDestination(const Station & s)
{
    destination_ = s;
    initDest();
    pubNoCurrentLoc();
    state_ = sMoving;
}

void RoutePlanner::run()
{
    switch( state_ )
    {
    case sUninit:
        sp_.knownStations().print();
        currentStation_ = sp_.knownStations().prompt("Current station? ");
        pubCurrentLoc();
        state_ = sIdle;
        break;

    case sIdle:
        sleep(1);
        break;

    case sMoving:
        if( goToDest() ) {
            state_ = sIdle;
            currentStation_ = destination_;
            pubCurrentLoc();
        }
        else {
            usleep(100000);
        }
        break;
    }
}
