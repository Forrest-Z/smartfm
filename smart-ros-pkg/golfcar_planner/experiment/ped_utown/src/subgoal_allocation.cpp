/*
 * subgoal_allocation.cpp
 *
 *  Created on: Sep 17, 2013
 *      Author: liuwlz
 */

#include "subgoal_allocation.h"

TempGoal::TempGoal(const int start, const int end){
	planning_horizon = 6.0;
	initDestination(start, end);
}

TempGoal::~TempGoal(){

}

int TempGoal::getRobotGlobalPose(){
	global_pose.setIdentity();
	tf::Stamped<tf::Pose> local_pose;
	local_pose.setIdentity();
	local_pose.frame_id_ = "/base_link";
	local_pose.stamp_ = ros::Time();
	ros::Time current_time = ros::Time::now(); // save time for checking tf delay later
	try {
		tf_.transformPose("/map", local_pose, global_pose);
	}
	catch(tf::LookupException& ex) {
		ROS_ERROR("No Transform available Error: %s\n", ex.what());
		return 0;
	}
	catch(tf::ConnectivityException& ex) {
		ROS_ERROR("Connectivity Error: %s\n", ex.what());
		return 0;
	}
	catch(tf::ExtrapolationException& ex) {
		ROS_ERROR("Extrapolation Error: %s\n", ex.what());
		return 0;
	}
	robot_pose.pose.position.x = global_pose.getOrigin().x();
	robot_pose.pose.position.y = global_pose.getOrigin().y();
	return 1;
}

int TempGoal::TransformToLocalMap(geometry_msgs::PoseStamped map_pose, geometry_msgs::PoseStamped &local_pose){
	try{
		tf_.waitForTransform("/local_map","/map", ros::Time::now(), ros::Duration(1.0));
		tf_.transformPose("local_map",map_pose, local_pose);
	}
	catch(tf::LookupException& ex) {
		ROS_ERROR("No Transform available Error: %s\n", ex.what());
		return 0;
	}
	catch(tf::ConnectivityException& ex) {
		ROS_ERROR("Connectivity Error: %s\n", ex.what());
		return 0;
	}
	catch(tf::ExtrapolationException& ex) {
		ROS_ERROR("Extrapolation Error: %s\n", ex.what());
		return 0;
	}
	return 1;
}

int TempGoal::initDestination(int start, int end){
	ROS_INFO("Initialize Destination and Got waypoints");
    destination_ = sp_.knownStations()(end);
    station_path_ = sp_.getPath(sp_.knownStations()(start),sp_.knownStations()(end));
    waypointNo_ = 0;
    global_path_.poses.resize(station_path_.size());
    for( unsigned i=0; i<station_path_.size(); i++ ){
    	global_path_.poses[i].pose.position.x = station_path_[i].x_;
    	global_path_.poses[i].pose.position.y = station_path_[i].y_;
    	global_path_.poses[i].pose.orientation.w = 1.0;
    	global_path_.poses[i].header.frame_id = "/map";
    	global_path_.poses[i].header.stamp = ros::Time::now();
    }
    if (global_path_.poses.size() == 0)
    	return 0;
	return 1;
}

int TempGoal::getNearestWaypoint(geometry_msgs::PoseStamped &waypoint, double horizon){
    //ROS_INFO_THROTTLE(3, "Going to %s. Distance=%.0f.", destination_.c_str(), distanceToGoal());
	ROS_INFO("Check Nearest Waypoint");
	while (getRobotGlobalPose() == 0){
		ros::spinOnce();
		ROS_INFO("TempGoal: Wait for robot pose");
	}
    double min_dist = DBL_MAX;
    for( unsigned i=0; i<station_path_.size(); i++ ){
        double dist = fmutil::distance(robot_pose.pose.position.x, robot_pose.pose.position.y, station_path_[i].x_, station_path_[i].y_);
        if(dist<min_dist){
            waypointNo_ = i;
            min_dist = dist;
        }
    }
    //Increment it to make sure the waypoint is in front
    double dist=0.0;
    do{
        waypointNo_++;
        dist = fmutil::distance(robot_pose.pose.position.x, robot_pose.pose.position.y, station_path_[waypointNo_].x_, station_path_[waypointNo_].y_);
    }while(dist<horizon);

	return 1;
}

/*
 * Two approaches to get reasonable temp goal:
 * 1) Control the planning horizon based on the degree of crowd (getAdaptiveSubgoal),
 * 2) Reason the dist_map/cost_map and find the most safe point (getSafeSubgoal).
 */

//TODO: Reason the best subgoal based on cost map, and tested
int TempGoal::getSafeSubgoal(geometry_msgs::PoseStamped &sub_goal){
	geometry_msgs::PoseStamped map_pose, local_pose;
	getNearestWaypoint(map_pose, planning_horizon);
	if (TransformToLocalMap(map_pose,local_pose) > 0){
		sub_goal = local_pose;
		return 1;
	}
	else
		return 0;
}

int TempGoal::getAdaptiveSubgoal(geometry_msgs::PoseStamped &sub_goal){
	ROS_INFO("get Adaptive subgoal");
	double adaptive_horizon = 6.0;
	geometry_msgs::PoseStamped map_pose, local_pose;
	getNearestWaypoint(map_pose, adaptive_horizon);
	sub_goal = map_pose;
	return 1;
	/*
	if (TransformToLocalMap(map_pose,local_pose) > 0){
		sub_goal = local_pose;
		return 1;
	}
	else
		return 0;
		*/
}


