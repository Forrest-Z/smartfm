#ifndef __ROUTE_PLANNER_VEHICLE__H__
#define __ROUTE_PLANNER_VEHICLE__H__

#include <math.h>

#include <string>
#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "route_planner.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


/// Drives the vehicle from A to B by feeding it a sequence of waypoints.
class RoutePlannerVehicle : public ROSRoutePlanner
{
public:
    RoutePlannerVehicle(StationPaths & sp);

private:
    ros::Publisher waypoint_pub_;
    ros::Publisher g_plan_pub_;
    ros::Publisher pointCloud_pub_;
    ros::Publisher nextpose_pub_;

    tf::TransformListener tf_;
    tf::Stamped<tf::Pose> global_pose_;

    MoveBaseClient ac_;
    StationPath path_;

    unsigned waypointNo_;

    bool goToDest();
    void initDest();

    void pubPathVis();
    void publishGoal();
    bool getRobotGlobalPose();
    void transformMapToOdom(geometry_msgs::PoseStamped *map_pose,
                            geometry_msgs::PointStamped *odom_point);
    double distanceToGoal();
};

#endif
