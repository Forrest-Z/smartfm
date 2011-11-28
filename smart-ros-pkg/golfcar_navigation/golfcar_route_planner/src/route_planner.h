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
    StationPaths & sp_;
    Station currentStation_, destination_;
    State state_;

    virtual void initDest() = 0;
    virtual bool goToDest() = 0;
    void run();
};


class DummyRoutePlanner : public RoutePlanner
{
public:
    DummyRoutePlanner(StationPaths & sp) : RoutePlanner(sp) { }
protected:
    bool goToDest() { return true; }
    void initDest() { }
};

#endif
