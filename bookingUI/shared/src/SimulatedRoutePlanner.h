#ifndef __SIMULATED_ROUTE_PLANNER__H__
#define __SIMULATED_ROUTE_PLANNER__H__

#include "RoutePlanner.h"

/// Simulates the vehicle
class SimulatedRoutePlanner : public RoutePlanner
{
    /// Controls the vehicle's speed (in m/s).
    /// If 0: the vehicle moves instantly to its destination.
    double vehicle_speed;

    /// odometry
    double distance_travelled;

    /// length of current path
    double distance_to_travel;

    /// time of last update
    double last_time, last_disp_time;

public:
    SimulatedRoutePlanner(const StationPaths & sp, float vehicle_speed);

    bool verbose;

protected:
    bool goToDest();
    void initDest();
};

#endif
