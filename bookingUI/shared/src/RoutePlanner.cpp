#include <stdlib.h>

#include <iostream>
#include <stdexcept>

#include "RoutePlanner.h"

using namespace std;



RoutePlanner::RoutePlanner(const StationPaths & sp) : sp_(sp)
{
    state_ = sUninit;
    latitude_ = 0.0;
    longitude_ = 0.0;
    eta_ = -1.0;
}

void RoutePlanner::setPath(const Station & start, const Station & end)
{
    if( state_==sUninit || state_==sReached ) {
        initDest(start, end);
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
        if( goToDest() ) state_ = sReached;
        else usleep(100000);
        break;
    }
}
