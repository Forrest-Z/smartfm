/*
 * ObstAvoid.h
 *
 *  Created on: Nov 19, 2013
 *      Author: liuwlz
 */

#ifndef OBSTAVOID_H_
#define OBSTAVOID_H_

#include <string>

#include <ros/ros.h>
#include <pnc_msgs/move_status.h>
#include <rrts_exp/rrts_status.h>
#include <nav_msgs/Path.h>

#include <Automoton_Control/ObstAvoid/StateMachine.hpp>

//TODO: Fix the tf problem and pass it down to other nodes;
namespace MPAV{

	class ObstAvoid{

		ObstAvoidSM navi_sm;

		ros::NodeHandle nh;
		ros::Subscriber move_status_sub;
		ros::Subscriber rrts_status_sub;
		ros::Subscriber global_plan_sub;
		ros::Subscriber rrts_plan_sub;
		ros::Subscriber global_goal_sub;
		ros::Subscriber cp_warn_sub;
		ros::Subscriber inter_sig_sub;

		ros::Publisher hybrid_plan_pub;
		ros::Publisher move_status_hybrid_pub;
		ros::Publisher station_goal_pub;

		ros::Timer status_check_timer;
		ros::Timer hybrid_path_timer;

		string global_frame, local_frame, base_frame;

		nav_msgs::Path reference_path, replan_path;
		nav_msgs::Path hybrid_path;
		geometry_msgs::PoseStamped stationGoal;

		enum Events{ObstDetect=0, ReachTemp, ReachDest, RRTSPathFound, RRTSPathUnsafe, RefPathFound};

		bool got_station_goal, init_hybrid_path, obst_avoid_trigger;

	public:
		ObstAvoid();
		~ObstAvoid();

		void GoalDistCheckTimer(const ros::TimerEvent &e);
		void MakePlanTimer(const ros::TimerEvent &e);
		void MoveStatusCB(const pnc_msgs::move_status::ConstPtr &move_status);
		void RRTSStatusCB(const rrts_exp::rrts_status::ConstPtr &rrts_status);
		void StationGoalCB(const geometry_msgs::PoseStamped global_goal);
		void RefPathCB(const nav_msgs::Path global_path);
		void RRTSPathCB(const nav_msgs::Path rrts_path);
		void ReplanTriggerCB(const std_msgs::Bool replan);
		void InterTriggerCB(const std_msgs::Bool inter);
		void CopyMoveStatus(const pnc_msgs::move_status status);

		friend struct RRTSPlan;
	};
}


#endif /* OBSTAVOID_H_ */
