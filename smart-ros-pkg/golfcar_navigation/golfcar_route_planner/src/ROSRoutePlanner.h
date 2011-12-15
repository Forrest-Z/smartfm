#ifndef __ROS_ROUTE_PLANNER__H__
#define __ROS_ROUTE_PLANNER__H__

#include <ros/ros.h>

#include <StationPath.h>
#include <RoutePlanner.h>

class ROSRoutePlanner : public RoutePlanner
{
public:
    ROSRoutePlanner(StationPaths & sp);

protected:
    ros::NodeHandle n;
    ros::Publisher pub;

    void pubCurrentLoc();
    void pubNoCurrentLoc();
};




#endif
