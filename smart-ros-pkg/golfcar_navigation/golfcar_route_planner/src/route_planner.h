#ifndef __ROUTE_PLANNER__H__
#define __ROUTE_PLANNER__H__


#include <golfcar_route_planner/station_path.h>

#include "threaded.h"


class MissionStateMachine;
class DBMissionComm;

/// A base class for route planners
class RoutePlanner : public Threaded
{
    friend class MissionStateMachine;
    friend class DBMissionComm;

public:
    RoutePlanner(StationPaths & sp) : sp_(sp), state_(sIdle) { }
    void setDestination(const Station & s);
    bool hasReached() const { return state_ == sIdle; }


protected:
    enum State { sUninit, sIdle, sMoving };

    StationPaths & sp_;
    Station currentStation_, destination_;
    State state_;

    virtual void initDest() = 0;
    virtual bool goToDest() = 0;
    void run();
};


class DummyRoutePlanner : public RoutePlanner
{
    DummyRoutePlanner(StationPaths & sp) : RoutePlanner(sp) { }
protected:
    bool goToDest() { return true; }
    void initDest() { }
};

#endif
