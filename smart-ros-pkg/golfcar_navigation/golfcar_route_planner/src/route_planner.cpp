#include "route_planner.h"

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
        if( sp_.knownStations().exists(currentStation_) )
            state_ = sIdle;
        ros::Duration(1).sleep();
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
