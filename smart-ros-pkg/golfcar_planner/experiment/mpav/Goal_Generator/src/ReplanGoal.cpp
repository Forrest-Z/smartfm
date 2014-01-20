/*
 * ReplanGoal.cpp
 *
 *  Created on: Nov 29, 2013
 *      Author: liuwlz
 */

#include <Goal_Generator/ReplanGoal.hpp>

namespace MPAV{
	ReplanGoal::ReplanGoal(const nav_msgs::Path global_path, double horizon)
	:Goal(){
		initReferencePath(global_path);
		subgoal_ready = false;
		goal_collision = false;
		planning_horizon = horizon;
	}

	ReplanGoal::~ReplanGoal(){

	}

	bool ReplanGoal::getNearWaypoint(){
		getNearestWaypoint();
		makeSubGoal();
		return true;
	}

	bool ReplanGoal::getNearWaypoint(geometry_msgs::PoseStamped poseIn){
		getNearestWaypoint(poseIn);
		makeSubGoal();
		return true;
	}

	int ReplanGoal::getSubgoal(geometry_msgs::PoseStamped &goal){
		getNearWaypoint();
		if (subgoal_ready){
			goal = sub_goal;
			subgoal_ready = false;
			return 1;
		}
		else
			return 0;
	}

	int ReplanGoal::getSubgoal(geometry_msgs::PoseStamped poseIn, geometry_msgs::PoseStamped &goal){
		getNearWaypoint(poseIn);
		if (subgoal_ready){
			goal = sub_goal;
			subgoal_ready = false;
			return 1;
		}
		else
			return 0;
	}

	int ReplanGoal::makeSubGoal(){
		double dist=0.0;
		do{
			assert(reference_path.poses.size()!=0);
			dist += Distance(reference_path.poses[waypointNo].pose.position.x, reference_path.poses[waypointNo].pose.position.y,
					reference_path.poses[waypointNo+1].pose.position.x, reference_path.poses[waypointNo+1].pose.position.y);
			waypointNo ++;
			if (waypointNo > reference_path.poses.size()-2){
				waypointNo = reference_path.poses.size() -2 ;
				break;
			}
		}while(dist<planning_horizon);

		ROS_INFO("Initialized waypoint %d", waypointNo);

		if(goal_collision){
			double dist = Distance(global_pose.getOrigin().x(), global_pose.getOrigin().y(), reference_path.poses[waypointNo].pose.position.x, reference_path.poses[waypointNo].pose.position.y);
			if(dist < 12){
				if(waypointNo < reference_path.poses.size()){
					waypointNo ++;
					ROS_INFO("Goal in collision reported, increment waypoint");
				}
			}
		}
		sub_goal.pose.position.x = reference_path.poses[waypointNo].pose.position.x;
		sub_goal.pose.position.y = reference_path.poses[waypointNo].pose.position.y;
		double map_yaw = 0;
		if( waypointNo < reference_path.poses.size()-1 ) {
			map_yaw = atan2(reference_path.poses[waypointNo+1].pose.position.y - reference_path.poses[waypointNo].pose.position.y, reference_path.poses[waypointNo+1].pose.position.x - reference_path.poses[waypointNo].pose.position.x);
		}
		else {
			assert(waypointNo!=0);
			map_yaw = atan2(reference_path.poses[waypointNo].pose.position.y - reference_path.poses[waypointNo-1].pose.position.y, reference_path.poses[waypointNo].pose.position.x - reference_path.poses[waypointNo-1].pose.position.x);
		}
		sub_goal.pose.orientation = tf::createQuaternionMsgFromYaw(map_yaw);
		sub_goal.header.frame_id=global_frame;
		sub_goal.header.stamp=ros::Time::now();
		subgoal_ready = true;
		return 1;
	}

	int ReplanGoal::getDist2SubGoal(double &dist2subgoal){
		getRobotGlobalPose();
		dist2subgoal = Distance(global_pose.getOrigin().x(), global_pose.getOrigin().y(),
				sub_goal.pose.position.x, sub_goal.pose.position.y);
		return 1;
	}
}

