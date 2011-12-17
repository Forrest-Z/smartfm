#ifndef __ROUTE_PLANNER__H__
#define __ROUTE_PLANNER__H__

#include "Threaded.h"
#include "StationPath.h"

/// A base class for route planners
class RoutePlanner : public Threaded
{
    friend class MissionComm;

public:
    RoutePlanner(StationPaths & sp);
    void setDestination(const Station & s);
    bool hasReached() const { return state_ == sIdle; }


protected:
    enum State { sIdle, sMoving };

    StationPaths & sp_;
    Station currentStation_, destination_;
    State state_;

    float latitude_, longitude_;
    float eta_;

    /// Called when the destination is received. Loads the path, etc. ...
    virtual void initDest() = 0;

    /// Called at each loop step. Performs the task of moving the vehicle.
    /// Returns whether the destination has been reached yet.
    virtual bool goToDest() = 0;

    void run();
};

#endif
