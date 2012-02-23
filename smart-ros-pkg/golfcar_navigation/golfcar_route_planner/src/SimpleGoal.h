#ifndef __ROUTE_PLANNER_VEHICLE_SIMPLE_GOAL__H__
#define __ROUTE_PLANNER_VEHICLE_SIMPLE_GOAL__H__

#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pnc_msgs/speed_contribute.h>
#include <map_to_world/coordinate.h>

#include "RoutePlanner.h"


/// Drives the vehicle from A to B by sending origin and destination station
/// number to the global planner:
/// publish a geometry_msgs::PoseStamped on "move_base_simple/goal"
class SimpleGoal : public RoutePlanner
{
public:
    SimpleGoal(const StationPaths & sp);

private:
    ros::NodeHandle nh;
    ros::Publisher goal_pub_;
    ros::Subscriber speed_status_sub_;
    ros::Subscriber world_coord_sub_;
    tf::TransformListener tf_;

    bool has_reached_;

    bool goToDest();
    void initDest();
    void speedStatusCallBack(const pnc_msgs::speed_contribute &);
    void worldCoordCallBack(const map_to_world::coordinate &);
};

#endif
