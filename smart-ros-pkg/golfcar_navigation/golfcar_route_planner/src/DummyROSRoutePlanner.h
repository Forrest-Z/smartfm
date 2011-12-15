#ifndef __DUMMY_ROS_ROUTE_PLANNER__H__
#define __DUMMY_ROS_ROUTE_PLANNER__H__

#include "ROSRoutePlanner.h"

/// Simulates the vehicle
class DummyROSRoutePlanner : public ROSRoutePlanner
{
    /// A ROS parameter to control the vehicle's speed (in m/s).
    /// If 0: the vehicle moves instantly to its destination. This is the
    /// default behavior.
    double vehicle_speed;

    /// odometry
    double distance_travelled;

    /// length of current path
    double distance_to_travel;

    /// time of last update
    double last_time;

public:
    DummyROSRoutePlanner(StationPaths & sp);

protected:
    bool goToDest();
    void initDest();
};

#endif
