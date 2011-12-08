#include <std_msgs/String.h>

#include "route_planner.h"

RoutePlanner::RoutePlanner(StationPaths & sp)
    : sp_(sp), state_(sIdle)
{
    pub = n.advertise<std_msgs::String>("missions/currentLocation", 1);

    sp_.knownStations().print();
    currentStation_ = sp_.knownStations().prompt("Current station? ");
    pubCurrentLoc();

    startThread();
}

void RoutePlanner::pubCurrentLoc()
{
    std_msgs::String s;
    s.data = currentStation_.str();
    pub.publish(s);
}

void RoutePlanner::pubNoCurrentLoc()
{
    std_msgs::String s;
    s.data = "";
    pub.publish(s);
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
        state_ = sIdle;
        break;

    case sIdle:
        ros::Duration(1).sleep();
        break;

    case sMoving:
        if( goToDest() ) {
            state_ = sIdle;
            currentStation_ = destination_;
            pubCurrentLoc();
        }
        else {
            ros::Duration(0.1).sleep();
        }
        break;
    }
}
