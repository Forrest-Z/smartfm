/*
 * rrts_replan.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: liuwlz
 */


#include "rrts_replan.h"

LocalReplanner::LocalReplanner(){

	root_received = false;
	goal_received = false;
	map_received = false;
	rrtstar = new rrts();
	local_map_sub = nh.subscribe("local_map", 10, &LocalReplanner::local_map_Callback, this);
}

LocalReplanner::~LocalReplanner(){

}

Pose LocalReplanner::local_map_transform(Pose transform_pose){

	tf::StampedTransform transform;
	Pose transformed_pose;
	try{
		tf_.transformPose("/base_link", transformed_pose, transform_pose);
	}
	catch (tf::TransformException ex) {
	    ROS_ERROR("%s",ex.what());
	}
	return transformed_pose;
}

void LocalReplanner::local_map_Callback(const pnc_msgs::local_map lm ){
	local_map = lm.occupancy;
	free_cell = lm.free_cells;
	int height = local_map.info.height;
	int width = local_map.info.width;
	double resolution = local_map.info.resolution;
	rrtstar->setBound(width, height);
}

void LocalReplanner::get_sub_goal(Pose sub_goal){
	goal_ = local_map_transform(sub_goal);

	double roll, pitch, yaw;
	tf::Quaternion q;
	tf::quaternionMsgToTF(goal_.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	goal[0] = goal_.pose.position.x;
	goal[1] = goal_.pose.position.y;
	goal[2] = yaw;

	rrtstar->setGoal(goal);
	goal_received = true;
}

void LocalReplanner::get_robot_pose(Pose robot_pose){
	root_ = local_map_transform(robot_pose);

	double roll, pitch, yaw;
	tf::Quaternion q;
	tf::quaternionMsgToTF(root_.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	root[0] = root_.pose.position.x;
	root[1] = root_.pose.position.y;
	root[2] = yaw;

	rrtstar->setRoot(root);
	root_received = true;
}

int LocalReplanner::get_trajectory(){

	rrtstar->planning();

	return 1;
}
