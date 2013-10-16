/*
 * rrts_replan.h
 *
 *  Created on: Sep 17, 2013
 *      Author: liuwlz
 */

#ifndef RRTS_REPLAN_H_
#define RRTS_REPLAN_H_

#include <ros/ros.h>
#include <pnc_msgs/local_map.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <rrts_exp/rrts_param.h>
#include <rrts_exp/rrts_status.h>
#include "subgoal_allocation.h"
#include <rrts_exp.hpp>
#include <pnc_msgs/local_map.h>
#include <dubins_car_exp.h>
#include <std_msgs/Bool.h>

typedef DubinsCarExp::StateTypeExp state_t;
typedef DubinsCarExp::TrajectoryTypeExp trajectory_t;
typedef DubinsCarExp::SystemTypeExp system_t;

typedef RRTstarExp::VertexExp <DubinsCarExp> vertex_t;
typedef RRTstarExp::RRTSPlanner <DubinsCarExp> planner_t;

using namespace std;

class RRTSReplan{

	planner_t rrtstar;
	system_t system;

	geometry_msgs::Point32 goal;
	nav_msgs::OccupancyGrid map;
	list<double*> committed_traj;
	list<float> committed_control;
	nav_msgs::Path rrts_traj;

	bool got_goal, got_map;
	double planner_dt;

public:
	RRTSReplan();
	~RRTSReplan();

	//ROS
	ros::NodeHandle nh_;
	ros::Subscriber local_map_sub_;
	ros::Publisher rrts_traj_pub_, rrts_status_pub_, rrts_tree_pub_, rrts_vertex_pub_;
	ros::Timer planner_timer, tree_pub_timer;

	geometry_msgs::Point32 robot_pose;

	tf::TransformListener tf_;
	rrts_exp::rrts_status status;
	nav_msgs::Path traj_msg;
	rrts_exp::rrts_status rrts_status;

	int Get_Robot_Pose();
	int Set_Goal_Region();
	void Get_Map(const pnc_msgs::local_map::ConstPtr local_map);
	void Get_Goal(geometry_msgs::PoseStamped sub_goal);
	int Setup();
	int Initalize();
	int Replan();
	int Safe_React();
	bool Robot_In_Goal();
	bool Robot_In_Collision();
	int Publish_Committed_Traj();
	int Publish_RRTS_Status();
	void Planner_Timer(const ros::TimerEvent &e);
	void Tree_Pub_Timer(const ros::TimerEvent &e);
	void Smach_Timer(const ros::TimerEvent &e);
};

#endif /* RRTS_REPLAN_H_ */
