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
#include <station_path.h>
#include "../src/RoutePlanner.hh"
#include <iostream>

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

	ros::Timer timer_;
	tf::TransformListener tf_;
	void waypoint_pub_loop();
	bool getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const;
	int distance_to_goal();
	std::vector<geometry_msgs::Point> targets_;
	int WaypointNo_;
	//assume that the vehicle will always stop at one of the station
	int currentStationID_;
	tf::Stamped<tf::Pose> global_pose;
};
};
