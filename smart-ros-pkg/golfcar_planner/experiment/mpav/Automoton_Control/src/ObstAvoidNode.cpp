/*
 * ObstAvoid.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: liuwlz
 */

#include <Automoton_Control/ObstAvoid/ObstAvoidNode.hpp>

namespace MPAV{

	ObstAvoid::ObstAvoid(){

		ROS_INFO("Enter ObstAvoid StateMachine");
		nh.param("base_frame", base_frame, string("base_link"));
		nh.param("global_frame", global_frame, string("map"));
		nh.param("local_frame", local_frame, string("local_map"));

		navi_sm.RRTStar = new PlannerExp();
		navi_sm.RRTStar->base_frame = base_frame;
		navi_sm.RRTStar->local_frame = local_frame;
		navi_sm.RRTStar->global_frame = global_frame;
		navi_sm.RRTStar->planner_timer.stop();

		navi_sm.SpeedControl = new SMSpeedControl();
		navi_sm.SpeedControl->base_frame = base_frame;

		navi_sm.SubGoal = NULL;

		/**
		 * Initialize the state machine and related objects
		 */
		navi_sm.initiate();
		navi_sm.move_status.dist_to_goal = DBL_MAX;
		for (size_t i = 0 ; i < 5; i++)
			navi_sm.SMStatus[i] =false;

		move_status_sub = nh.subscribe("move_status_repub", 1, &ObstAvoid::MoveStatusCB, this);
		rrts_status_sub = nh.subscribe("rrts_status",1, &ObstAvoid::RRTSStatusCB, this);
		global_plan_sub = nh.subscribe("global_plan", 1, &ObstAvoid::RefPathCB, this);
		rrts_plan_sub = nh.subscribe("pnc_trajectory",1,&ObstAvoid::RRTSPathCB, this);
		global_goal_sub = nh.subscribe("sm_station_goal",1,&ObstAvoid::StationGoalCB, this);
		cp_warn_sub = nh.subscribe("replan_trigger",1,&ObstAvoid::ReplanTriggerCB, this);
		//TODO: TBD
		inter_sig_sub = nh.subscribe("inter_trigger",1,&ObstAvoid::InterTriggerCB, this);

		move_status_hybrid_pub = nh.advertise<pnc_msgs::move_status>("move_status_hybrid",1);
		hybrid_plan_pub = nh.advertise<nav_msgs::Path>("global_plan_repub", 1);
		station_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);

		status_check_timer = nh.createTimer(ros::Duration(0.1), &ObstAvoid::GoalDistCheckTimer, this);
		hybrid_path_timer = nh.createTimer(ros::Duration(0.1), &ObstAvoid::MakePlanTimer, this);

		got_station_goal = false;
		init_hybrid_path = false;

		ros::MultiThreadedSpinner sprinner(1);
		sprinner.spin();
	}

	ObstAvoid::~ObstAvoid(){

	}

	void ObstAvoid::GoalDistCheckTimer(const ros::TimerEvent& e){
		if(navi_sm.SubGoal == NULL)
			return;
		double dist_sub_goal;
		navi_sm.SubGoal->getDist2SubGoal(dist_sub_goal);
		//ROS_INFO("Dist to Dest: %f, Dist to SubGoal: %f", navi_sm.move_status.dist_to_goal, dist_sub_goal);
		if (dist_sub_goal<2.5){
			navi_sm.SMStatus[ReachTemp] = true;
			navi_sm.process_event(Ev_Reach_Temp_Goal());
		}

		if (navi_sm.move_status.dist_to_goal < 3.0){
			navi_sm.SMStatus[ReachDest] = true;
			navi_sm.process_event(Ev_Reach_Dest());
		}
	}

	void ObstAvoid::RRTSStatusCB(const rrts_exp::rrts_status::ConstPtr &rrts_status){
		navi_sm.SMStatus[RRTSPathFound] = rrts_status->trajectory_found;
		if (navi_sm.SMStatus[RRTSPathFound])
			navi_sm.process_event(Ev_RRTSPath_Found());
		else{
			navi_sm.process_event(Ev_RRTSPath_Unsafe());
		}
	}

#if 1
	void ObstAvoid::MoveStatusCB(const pnc_msgs::move_status::ConstPtr &move_status){
		navi_sm.SMStatus[ObstDetect] = move_status->emergency;
		if (move_status->emergency && !navi_sm.RRTStar->is_first_map)
			navi_sm.process_event(Ev_Obst_Detected());
		CopyMoveStatus(*move_status);
		move_status_hybrid_pub.publish(navi_sm.move_status);
	}

	void ObstAvoid::ReplanTriggerCB(const std_msgs::Bool replan){
		if (replan.data == 1)
			ROS_INFO("Warning");
	}

	void ObstAvoid::InterTriggerCB(const std_msgs::Bool inter){
		if (inter.data == 1 && navi_sm.state_downcast<const NormalPlan*>() != 0)
			navi_sm.process_event(Ev_Enter_Intersection());
		if (inter.data == 0 && navi_sm.state_downcast<const Intersection*>() != 0)
			navi_sm.process_event(Ev_Intersection_Safe());
	}

#else if
	void ObstAvoid::ReplanTriggerCB(const std_msgs::Bool replan){
		if (replan.data == 1)
			ROS_INFO("Warning");
		if (replan.data == 1 && !navi_sm.RRTStar->is_first_map){
			navi_sm.process_event(Ev_Obst_Detected());
		}
	}

	void ObstAvoid::MoveStatusCB(const pnc_msgs::move_status::ConstPtr &move_status){
		navi_sm.SMStatus[ObstDetect] = move_status->emergency;
		CopyMoveStatus(*move_status);
		move_status_hybrid_pub.publish(navi_sm.move_status);
	}
#endif

	void ObstAvoid::RefPathCB(const nav_msgs::Path global_path){
		reference_path = global_path;
		navi_sm.SMStatus[RefPathFound] = true;
		navi_sm.SubGoal = new ReplanGoal(reference_path, 12);
		navi_sm.SubGoal->base_frame = base_frame;
		navi_sm.SubGoal->local_frame = local_frame;
		navi_sm.SubGoal->global_frame = global_frame;

		if (!init_hybrid_path){
			hybrid_path = reference_path;
			init_hybrid_path = true;
		}
		navi_sm.process_event(Ev_PrePath_Got());
	}

	void ObstAvoid::RRTSPathCB(const nav_msgs::Path rrts_path){
		replan_path = rrts_path;
		navi_sm.SMStatus[RRTSPathFound] = true;
	}

	void ObstAvoid::StationGoalCB(const geometry_msgs::PoseStamped global_goal){
		stationGoal = global_goal;
		got_station_goal = true;
	}

	void ObstAvoid::CopyMoveStatus(const pnc_msgs::move_status status){
		navi_sm.move_status.dist_to_goal = status.dist_to_goal;
		navi_sm.move_status.dist_to_ints = status.dist_to_ints;
		navi_sm.move_status.dist_to_sig = status.dist_to_sig;
		navi_sm.move_status.backward_driving = status.backward_driving;
		navi_sm.move_status.want_exact_stop = status.want_exact_stop;
		navi_sm.move_status.sig_type = status.sig_type;
		navi_sm.move_status.obstacles_dist = status.obstacles_dist;
		navi_sm.move_status.int_point = status.int_point;
		navi_sm.move_status.steer_angle = status.steer_angle;
	}

	void ObstAvoid::MakePlanTimer(const ros::TimerEvent &e){
		if (navi_sm.state_downcast<const HeadSubGoal*>() != 0){
			hybrid_path.poses.clear();
			size_t local_temp_no = 0;
			size_t global_temp_no = navi_sm.SubGoal->getWaypointNo() + 1;
			hybrid_path.poses.clear();
			while (local_temp_no < replan_path.poses.size()){
				hybrid_path.poses.push_back(replan_path.poses[local_temp_no]);
				local_temp_no ++;
			}
			while (global_temp_no < reference_path.poses.size()){
				hybrid_path.poses.push_back(reference_path.poses[global_temp_no]);
				global_temp_no ++;
			}
		}

		hybrid_path.header.frame_id = global_frame;
		hybrid_path.header.stamp = ros::Time::now();
		hybrid_plan_pub.publish(hybrid_path);
		if (got_station_goal)
			station_goal_pub.publish(stationGoal);
	}
}
