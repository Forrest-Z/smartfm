/*
 * rrts_interface.h
 *
 *  Created on: Jan 24, 2013
 *      Author: liuwlz
 */

#ifndef RRTS_INTERFACE_H_
#define RRTS_INTERFACE_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <rrts_ompl/rrts_edge.h>

#include "rrtstar.h"

using namespace std;

class rrts_interface{

	ros::NodeHandle nh_;
	ros::Subscriber map_sub_;
	ros::Subscriber goal_sub_;
	tf::TransformListener tf_;

	rrts *rrtstar;

	nav_msgs::OccupancyGrid local_map;
	geometry_msgs::PoseStamped sub_goal;
	geometry_msgs::Point32 car_pose;
	tf::TransformListener goal_tf;
	float resolution;

	vector<double> path_reals;
	vector<rrts_ompl::rrts_edge> rrts_tree_visual;

public:
	rrts_interface();
	~rrts_interface();

	void goal_CB(const geometry_msgs::PoseStamped goal);
	void map_CB(const nav_msgs::OccupancyGrid map);
	int local_from_global();
	int global_from_local();
	int got_plan();
	int visualize();
};

#endif /* RRTS_INTERFACE_H_ */
