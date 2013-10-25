/*
 * subgoal_allocation.h
 *
 *  Created on: Sep 17, 2013
 *      Author: liuwlz
 */

#ifndef SUBGOAL_ALLOCATION_H_
#define SUBGOAL_ALLOCATION_H_

#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <svg_path/StationPath.h>
#include <fmutil/fm_math.h>

class TempGoal{

	//Station path
	StationPaths sp_;
    StationPath station_path_;
    Station destination_;

    tf::Stamped<tf::Pose> global_pose;
    geometry_msgs::PoseStamped robot_pose;
    nav_msgs::OccupancyGrid dist_map;

    //Parameter for max RRT* planning horizon
    double planning_horizon;

public:

	TempGoal(const int start, const int end);
	~TempGoal();

	tf::TransformListener tf_;
    nav_msgs::Path global_path_;
    int waypointNo_;

    int getRobotGlobalPose();
    int TransformToLocalMap(geometry_msgs::PoseStamped map_pose, geometry_msgs::PoseStamped &local_pose);
	int initDestination(int start, int end);
	int getNearestWaypoint(geometry_msgs::PoseStamped &waypoint,  double horizon);
	int getSafeSubgoal(geometry_msgs::PoseStamped &sub_goal);
	int getAdaptiveSubgoal(geometry_msgs::PoseStamped &sub_goal);
};

#endif /* SUBGOAL_ALLOCATION_H_ */
