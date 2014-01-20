/*
 * ReplanGoal.hpp
 *
 *  Created on: Nov 19, 2013
 *      Author: liuwlz
 */

#ifndef NEARESTGOAL_HPP_
#define NEARESTGOAL_HPP_

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <Goal_Generator/Goal.hpp>

#define NDEBUG

namespace MPAV{

	class ReplanGoal:public Goal{

		geometry_msgs::PoseStamped sub_goal;
		bool subgoal_ready, goal_collision;
		double planning_horizon;

	public:
		ReplanGoal(const nav_msgs::Path global_path, double horizon);
		~ReplanGoal();
		bool getNearWaypoint();
		bool getNearWaypoint(geometry_msgs::PoseStamped poseIn);
		int makeSubGoal();
		int getSubgoal(geometry_msgs::PoseStamped &goal);
		int getSubgoal(geometry_msgs::PoseStamped poseIn, geometry_msgs::PoseStamped &goal);
		int getDist2SubGoal(double &dist2subgoal);
	};
}

#endif /* NEARESTGOAL_HPP_ */
