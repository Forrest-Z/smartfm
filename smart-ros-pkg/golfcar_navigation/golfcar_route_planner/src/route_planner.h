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


/// Simulates the vehicle (moves instantaneously from A to B).
class DummyROSRoutePlanner : public ROSRoutePlanner
{
public:
    DummyROSRoutePlanner(StationPaths & sp) : ROSRoutePlanner(sp) { }
protected:
    bool goToDest()
    {
        ROS_INFO("Moving from %s to %s", currentStation_.c_str(), destination_.c_str());
        return true;
    }

    void initDest() { }
};

#endif
