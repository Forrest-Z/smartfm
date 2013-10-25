/*
 * ped_utown_node.h
 *
 *  Created on: Sep 24, 2013
 *      Author: liuwlz
 */

#ifndef PED_UTOWN_NODE_H_
#define PED_UTOWN_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <pnc_msgs/move_status_smach.h>
#include <pnc_msgs/move_status.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include "ped_utown/smachCTRConfig.h"


#include "rrts_replan.h"
#include "subgoal_allocation.h"

class PedNavi{
	TempGoal *temp_goal;
	RRTSReplan *planner;
	nav_msgs::Path hybrid_path, global_path, rrts_path;
	pnc_msgs::move_status_smach move_status_smach;
	pnc_msgs::move_status move_status_hybrid;

public:
	ros::NodeHandle nh;
	ros::Subscriber replan_cmd_sub, situation_check_sub, safe_react_sub, move_status_sub;
	ros::Publisher hybrid_plan_pub, situation_result_pub, move_status_pub, smach_status_pub;
	ros::Timer smach_timer;

	dynamic_reconfigure::Server<ped_utown::smachCTRConfig>  *srv;

	bool replan_cmd, situation_check_cmd, safe_react_cmd, rrts_is_planning;

	PedNavi(const int start, const int end);
	~PedNavi();

	void SmachDynamicReconfig(ped_utown::smachCTRConfig &config, uint32_t level);
	void ReplanCmd(std_msgs::Bool replan);
	void SituationCheckCmd(std_msgs::Bool situation_check);
	void SafeReactCmd(std_msgs::Bool safe_react);
	void MoveStatusCB(pnc_msgs::move_status move_status);
	void SmachInterface(const ros::TimerEvent &e);
	void HybridPlanPub();

	int Initialize();
	int Replan();
	int SafeAction();
	int SituationEvaluate();
};
#endif /* PED_UTOWN_NODE_H_ */
