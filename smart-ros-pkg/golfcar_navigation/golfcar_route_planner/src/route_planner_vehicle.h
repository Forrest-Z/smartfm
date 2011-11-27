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


class RoutePlannerVehicle : public RoutePlanner
{
public:
    RoutePlannerVehicle();
    void spin();

private:
    ros::NodeHandle n;
    ros::Publisher waypoint_pub_;
    ros::Publisher g_plan_pub_;
    ros::Publisher pointCloud_pub_;
    ros::Publisher poseStamped_pub_;
    ros::Publisher nextpose_pub_;
    ros::Subscriber gps_sub_;

    ros::Timer timer_;
    tf::TransformListener tf_;

    StationPaths sp_;
    StationPath targets_;

    tf::Stamped<tf::Pose> global_pose_;

    MoveBaseClient ac_;

    bool standalone_;
    bool debugMode_;
    unsigned waypointNo_;

    string usrID_;
    Station currentStationID_, dropoff_, pickup_;

    void pubWaypoint();
    void pubPathVis();
    void clearScreen();
    void publishGoal(const Station & start, const Station & end);
    bool getRobotGlobalPose();
    void transformMapToOdom(geometry_msgs::PoseStamped *map_pose,
                            geometry_msgs::PointStamped *odom_point);
    double distanceToGoal();
    void waypointLoop(VehicleStatus::Status,
                      const Station & start, const Station & end);
    void printStationList() const;
    Station promptForStation( const string & prompt ) const;
};

#endif
