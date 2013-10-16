/*
 * ped_utown_node.cpp
 *
 *  Created on: Sep 17, 2013
 *      Author: liuwlz
 */


#include "ped_utown_node.h"

PedNavi::PedNavi(const int start, const int end){

	temp_goal = new TempGoal(start, end);
	planner = new RRTSReplan();

	planner->planner_timer.stop();

	replan_cmd = false;
	situation_check_cmd = false;
	safe_react_cmd = false;
	rrts_is_planning = false;

	replan_cmd_sub = nh.subscribe("replan_cmd",1,&PedNavi::ReplanCmd, this);
	situation_check_sub = nh.subscribe("situation_check",2,&PedNavi::SituationCheckCmd,this);
	safe_react_sub = nh.subscribe("safe_react",2,&PedNavi::SafeReactCmd, this);
	move_status_sub = nh.subscribe("move_status_repub",2,&PedNavi::MoveStatusCB, this);

	smach_timer = nh.createTimer(ros::Duration(0.05),&PedNavi::SmachInterface,this);

	hybrid_plan_pub = nh.advertise<nav_msgs::Path>("hybrid_plan",2);
	smach_status_pub = nh.advertise<pnc_msgs::move_status_smach>("smach_status",1);
	move_status_pub = nh.advertise<pnc_msgs::move_status>("move_status_hybrid",1);

	srv = new dynamic_reconfigure::Server<ped_utown::smachCTRConfig> (ros::NodeHandle("~"));
	dynamic_reconfigure::Server<ped_utown::smachCTRConfig>::CallbackType f = boost::bind(&PedNavi::SmachDynamicReconfig, this, _1, _2);
	srv->setCallback(f);

	Initialize();
	ros::spin();
}

PedNavi::~PedNavi(){

}

void PedNavi::SmachDynamicReconfig(ped_utown::smachCTRConfig &config, uint32_t level){
	move_status_smach.emergency = config.emergency;
	move_status_smach.path_type = config.path_type;
	move_status_smach.global_path_exist = config.global_path_exist;
	move_status_smach.replan_path_exist = config.replan_path_exist;
	move_status_smach.reach_destination = config.reach_destination;
	move_status_smach.reach_temp_goal = config.reach_temp_goal;
	move_status_smach.interface_end = config.interface_end;
}

int PedNavi::Initialize(){
	/*
	 * Initialise the move_status_smach for interfacing with State Machine
	 */
	ROS_INFO("Initialize_Ped_Utown");

	global_path.poses = temp_goal->global_path_.poses;

	if (global_path.poses.size()!=0)
		move_status_smach.global_path_exist = true;
	else
		move_status_smach.global_path_exist = false;

	move_status_smach.emergency = false;
	move_status_smach.path_type = 1;
	move_status_smach.replan_path_exist = false;
	move_status_smach.reach_destination = false;
	move_status_smach.reach_temp_goal = false;
	move_status_smach.interface_end = false;
	return 1;
}

void PedNavi::ReplanCmd(std_msgs::Bool replan){
	replan_cmd = replan.data;
}

void PedNavi::SituationCheckCmd(std_msgs::Bool situation_check){
	situation_check_cmd = situation_check.data;
}

void PedNavi::SafeReactCmd(std_msgs::Bool safe_react){
	safe_react_cmd = safe_react.data;
}

void PedNavi::MoveStatusCB(pnc_msgs::move_status move_status){
	move_status_hybrid = move_status;
}

void PedNavi::SmachInterface(const ros::TimerEvent &e){
	ROS_INFO("Smach Interface");
	move_status_smach.emergency = move_status_hybrid.emergency;
	if (replan_cmd == true)
		Replan();
	else
		planner->planner_timer.stop();
	if (situation_check_cmd)
		SituationEvaluate();
	if (safe_react_cmd)
		SafeAction();
	smach_status_pub.publish(move_status_smach);
	move_status_pub.publish(move_status_hybrid);
}

int PedNavi::Replan(){
	//Generate the sub_goal
	//move_status_smach.emergency = false;
	ROS_INFO("RRTS_Replan");
	if (!rrts_is_planning){
		geometry_msgs::PoseStamped sub_goal;
		temp_goal->getAdaptiveSubgoal(sub_goal);
		planner->Get_Goal(sub_goal);
		planner->planner_timer.start();
		rrts_is_planning = true;
	}
	if (planner->rrts_status.trajectory_found){
		rrts_is_planning = false;
		replan_cmd = false;
		move_status_smach.replan_path_exist = true;
		rrts_path.poses = planner->traj_msg.poses;
	}
	return 1;
}

int PedNavi::SafeAction(){
	planner->Safe_React();
	move_status_hybrid.emergency = true;
	move_status_hybrid.path_exist = false;
	move_status_hybrid.want_exact_stop = true;
	return 1;
}

int PedNavi::SituationEvaluate(){

	//Reset the move_status for smach
	move_status_smach.emergency = false;
	move_status_smach.replan_path_exist = false;
	move_status_smach.interface_end = false;

	move_status_smach.path_type = 1;
	if (move_status_smach.path_type == 0){
		ROS_INFO("Follow Pre-defined path");
		hybrid_path.poses = global_path.poses;
		move_status_hybrid.emergency = false;
		move_status_hybrid.path_exist = true;
	}
	if (move_status_smach.path_type == 1){
		ROS_INFO("Follow RRTS path");
		hybrid_path.poses = rrts_path.poses;
		move_status_hybrid.emergency = false;
		move_status_hybrid.path_exist = true;
	}
	if (move_status_smach.path_type == 2){
		SafeAction();
		ROS_INFO("Waiting for Safety");
	}
	return 1;
}

void PedNavi::HybridPlanPub(){
	hybrid_path.header.frame_id = "/map";
	hybrid_path.header.stamp = ros::Time::now();
	hybrid_plan_pub.publish(hybrid_path);
}

int main(int argc, char**argv){
    ros::init(argc, argv, "ped_navi");
    if(argc<3)
        std::cout<<"Usage: route_planner start end"<<std::endl;
    else
        PedNavi pednavi(atoi(argv[1]), atoi(argv[2]));
	ros::spin();
    return 0;
}

