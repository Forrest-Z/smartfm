#include "route_planner.h"

RoutePlanner::RoutePlanner(StationPaths & sp)
    : sp_(sp), state_(sIdle)
{
    sp_.knownStations().print();
    currentStation_ = sp_.knownStations().prompt("Current station? ");
    startThread();
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
    case sUninit:
        state_ = sIdle;
        break;

    case sIdle:
        ros::Duration(1).sleep();
        break;

    case sMoving:
        if( goToDest() ) {
            state_ = sIdle;
            currentStation_ = destination_;
        }
        else {
            ros::Duration(0.1).sleep();
        }
        break;
    }
}
