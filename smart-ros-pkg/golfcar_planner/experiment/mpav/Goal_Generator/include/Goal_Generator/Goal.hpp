/*
 * Goal.hpp
 *
 *  Created on: Nov 21, 2013
 *      Author: liuwlz
 */

#ifndef GOAL_HPP_
#define GOAL_HPP_

#include <stdlib.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <MPAVUtil/Range.hpp>

using namespace std;

namespace MPAV{

	typedef geometry_msgs::PoseStamped Pose;

	class Goal{
	protected:

		tf::TransformListener tf_;
		tf::Stamped<tf::Pose> global_pose;

		nav_msgs::Path reference_path;

		Pose robot_pose;

		size_t waypointNo;
		int initReferencePath(const nav_msgs::Path global_path);
		int transformGoalPose(Pose poseIn, Pose &poseOut);
		int getRobotGlobalPose();
		int getNearestWaypoint();
		int getNearestWaypoint(geometry_msgs::PoseStamped PoseIn);

	public:

		string global_frame, local_frame, base_frame;

		Pose goal;
		vector<Pose> goal_candidate;

		Goal();
		~Goal();

		int getWaypointNo(){return waypointNo;}
		inline double Distance(double x_1, double y_1, double x_2, double y_2){
			return sqrt((x_2-x_1)*(x_2-x_1) + (y_2-y_1)*(y_2-y_1) );
		}
	};

	Goal::Goal(){
		waypointNo = 0;
		global_frame = "map";
		local_frame = "local_map";
		base_frame = "base_link";
	}

	Goal::~Goal(){

	}

	int Goal::initReferencePath(const nav_msgs::Path global_path){
		assert(global_path.poses.size()>0);
		reference_path = global_path;
		return 1;
	}

	int Goal::transformGoalPose(Pose poseIn, Pose &poseOut){
		poseIn.header.stamp = ros::Time();
		try{
			tf_.waitForTransform(global_frame, local_frame, ros::Time::now(), ros::Duration(1.0));
			tf_.transformPose(global_frame, poseIn, poseOut);
		}
		catch(tf::LookupException& ex) {
			ROS_ERROR("No Transform available Error: %s\n", ex.what());
			return false;
		}
		catch(tf::ConnectivityException& ex) {
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return false;
		}
		return true;
	}

	int Goal::getRobotGlobalPose(){
		global_pose.setIdentity();
		tf::Stamped<tf::Pose> robot_pose;
		robot_pose.setIdentity();
		robot_pose.frame_id_ = base_frame;
		robot_pose.stamp_ = ros::Time();
		try {
			tf_.transformPose(global_frame, robot_pose, global_pose);
		}
		catch(tf::LookupException& ex) {
			ROS_ERROR("No Transform available Error: %s\n", ex.what());
			return false;
		}
		catch(tf::ConnectivityException& ex) {
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return false;
		}
		return true;
	}

	int Goal::getNearestWaypoint(){
		ROS_INFO("Check Nearest Waypoint");
		ros::Rate loop_rate(3);
		while(!getRobotGlobalPose()){
			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO("Waiting for Robot pose");
		}
		double min_dist = DBL_MAX;
		for( size_t i=0; i<reference_path.poses.size(); i++ ){
			double dist = Distance(global_pose.getOrigin().x(), global_pose.getOrigin().y(),
					reference_path.poses[i].pose.position.x, reference_path.poses[i].pose.position.y);
			if(dist < min_dist){
				waypointNo = i;
				min_dist = dist;
			}
		}
		ROS_INFO("Nearest Waypoint No: %d", waypointNo);
		return 1;
	}

	int Goal::getNearestWaypoint(geometry_msgs::PoseStamped PoseIn){
		ROS_INFO("Check Nearest Waypoint for Pose: %f, %f", PoseIn.pose.position.x, PoseIn.pose.position.y);
		double min_dist = DBL_MAX;
		for( size_t i=0; i<reference_path.poses.size(); i++ ){
			double dist = Distance(PoseIn.pose.position.x, PoseIn.pose.position.y,
					reference_path.poses[i].pose.position.x, reference_path.poses[i].pose.position.y);
			if(dist < min_dist){
				waypointNo = i;
				min_dist = dist;
			}
		}
		ROS_INFO("Nearest Waypoint No: %d", waypointNo);
		return 1;
	}
}

#endif /* GOAL_HPP_ */
