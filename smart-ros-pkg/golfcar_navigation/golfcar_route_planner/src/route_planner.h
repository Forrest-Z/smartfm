#ifndef __ROUTE_PLANNER__H__
#define __ROUTE_PLANNER__H__

#include <ros/ros.h>
#include <golfcar_route_planner/station_path.h>

#include "threaded.h"


class DBMissionComm;

/// A base class for route planners
class RoutePlanner : protected Threaded
{
    friend class DBMissionComm;

public:
    RoutePlanner(StationPaths & sp);
    void setDestination(const Station & s);
    bool hasReached() const { return state_ == sIdle; }


protected:
    enum State { sUninit, sIdle, sMoving };

    ros::NodeHandle n;
    ros::Publisher pub;

    StationPaths & sp_;
    Station currentStation_, destination_;
    State state_;

    /// Called when the destination is received. Loads the path, etc. ...
    virtual void initDest() = 0;

    /// Called at each loop step. Performs the task of moving the vehicle.
    /// Returns whether the destination has been reached yet.
    virtual bool goToDest() = 0;

    void run();

private:
    void pubCurrentLoc();
    void pubNoCurrentLoc();
};


/// Simulates the vehicle (moves instantaneously from A to B).
class DummyRoutePlanner : public RoutePlanner
{
public:
    DummyRoutePlanner(StationPaths & sp) : RoutePlanner(sp) { }
protected:
    bool goToDest()
    {
        ROS_INFO("Moving from %s to %s", currentStation_.c_str(), destination_.c_str());
        return true;
    }

    void initDest() { }
};

#endif
