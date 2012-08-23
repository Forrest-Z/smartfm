#ifndef __ROUTE_PLANNER__H__
#define __ROUTE_PLANNER__H__

#include "Threaded.h"
#include "StationPath.h"

/// A base class for route planners
class RoutePlanner : public Threaded
{
public:
    explicit RoutePlanner(const StationPaths & sp);

    void setPath(const Station & start, const Station & end);
    bool hasReached() const { return state_ == sReached; }
    void start();

    float get_ETA() const { return eta_; }
    float get_lat() const { return latitude_; }
    float get_lon() const { return longitude_; }

    const StationPaths & sp_;

private:
    enum State { sUninit, sReady, sMoving, sReached };

    /// The current state
    State state_;

    /// The main thread function and the actual implementation of the state machine
    void run();

protected:
    double latitude_, longitude_;
    float eta_;

    /// Called when the destination is received. Loads the path, etc. ...
    virtual void initDest(const Station & start, const Station & end) = 0;

    /// Called at each loop step. Performs the task of moving the vehicle.
    /// Returns whether the destination has been reached yet.
    virtual bool goToDest() = 0;
};

#endif
