/*
 * route_planner.h
 *
 *  Created on: Jul 21, 2011
 *      Author: golfcar
 */

#ifndef ROUTE_PLANNER_H_
#define ROUTE_PLANNER_H_


#endif /* ROUTE_PLANNER_H_ */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include "station_path_sparse.h"
#include "../../src/RoutePlanner.hh"
#include <iostream>
#include <math.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
namespace route_planner {

class RoutePlannerNode
{
public:
	RoutePlannerNode();
	~RoutePlannerNode();

private:
	ros::Publisher waypoint_pub_;
	ros::Publisher g_plan_pub_;
	ros::Publisher pointCloud_pub_;
	ros::Publisher poseStamped_pub_;
	ros::Publisher nextpose_pub_;
	ros::Subscriber gps_sub_;

	ros::Timer timer_;
	tf::TransformListener tf_;
	int gpsCount_;
	void waypoint_pub();
	void publishPathVis();
	void clearscreen();
	void publish_goal(double pickup, double dropoff);
	bool getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const;
	void transformMapToOdom(geometry_msgs::PoseStamped &map_pose, geometry_msgs::PointStamped &odom_point);
	void gpsCallBack(const sensor_msgs::NavSatFixConstPtr& fix);
	int distance_to_goal();
	void startLoop(VehicleStatus vehstatus,int dropoff, int pickup,RoutePlanner *rp);
	void startLoop(VehicleStatus vehstatus,int dropoff, int pickup);
	std::vector<geometry_msgs::Point> targets_;
	RoutePlanner *rp_;
	int WaypointNo_;
	int LastPublishedPoint_;
	//assume that the vehicle will always stop at one of the station
	int currentStationID_;
	tf::Stamped<tf::Pose> global_pose;
	MoveBaseClient *ac_;
};
};
