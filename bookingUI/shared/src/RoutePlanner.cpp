#include <stdlib.h>

#include <iostream>

#include "RoutePlanner.h"
#include "MissionComm.h"

using namespace std;



RoutePlanner::RoutePlanner(StationPaths & sp)
    : sp_(sp), state_(sIdle), latitude_(0.0), longitude_(0.0), eta_(-1.0)
{

}

void RoutePlanner::setDestination(const Station & s)
{
    destination_ = s;
    initDest();
    state_ = sMoving;
}

void RoutePlanner::run()
{
    switch( state_ )
    {
    case sIdle:
        sleep(1);
        break;

    case sMoving:
        if( goToDest() ) {
            state_ = sIdle;
            currentStation_ = destination_;
        }
        else {
            usleep(100000);
        }
        break;
    }
}
