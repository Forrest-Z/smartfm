#include <std_msgs/String.h>

#include "ROSRoutePlanner.h"

ROSRoutePlanner::ROSRoutePlanner(StationPaths & sp)
    : RoutePlanner(sp)
{
    pub = n.advertise<std_msgs::String>("missions/currentLocation", 1);
}

void ROSRoutePlanner::pubCurrentLoc()
{
    std_msgs::String s;
    s.data = currentStation_.str();
    pub.publish(s);
}

void ROSRoutePlanner::pubNoCurrentLoc()
{
    std_msgs::String s;
    s.data = "";
    pub.publish(s);
}
