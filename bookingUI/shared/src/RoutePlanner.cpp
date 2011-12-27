#include <stdlib.h>

#include <iostream>
#include <stdexcept>

#include "RoutePlanner.h"
#include "MissionComm.h"

using namespace std;



RoutePlanner::RoutePlanner(const StationPaths & sp)
    : sp_(sp), state_(sUninit), latitude_(0.0), longitude_(0.0), eta_(-1.0)
{

}

void RoutePlanner::setDestination(const Station & s)
{
    if( state_==sUninit || state_==sReached ) {
        destination_ = s;
        initDest();
        state_ = sReady;
    }
    else {
        throw runtime_error("Attempting to set destination at the wrong time");
    }
}

void RoutePlanner::start()
{
    if( state_==sReady )
        state_ = sMoving;
    else
        throw runtime_error("Attempting to start when not ready");
}

void RoutePlanner::run()
{
    switch( state_ )
    {
    case sUninit:
    case sReady:
    case sReached:
        sleep(1);
        break;

    case sMoving:
        if( goToDest() ) {
            state_ = sReached;
            currentStation_ = destination_;
        }
        else {
            usleep(100000);
        }
        break;
    }
}
