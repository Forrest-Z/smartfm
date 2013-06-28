/*
 * rrts_replan.h
 *
 *  Created on: Apr 2, 2013
 *      Author: liuwlz
 */

#ifndef RRTS_REPLAN_H_
#define RRTS_REPLAN_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <pnc_msgs/local_map.h>

//#include "rrtstar.h"
#include "rrts_ompl/rrtstar.h"

typedef geometry_msgs::PoseStamped Pose;

class LocalReplanner{
	LocalReplanner();
	~LocalReplanner();

    //ros
	ros::NodeHandle nh;

    ros::Publisher rrts_status_pub;
    ros::Publisher vertex_pub;
    ros::Publisher tree_pub;
    ros::Publisher trajectory_pub;

    ros::Subscriber local_map_sub;

    ros::Timer rrts_status_timer;
    ros::Timer planner_timer;
    ros::Timer tree_pub_timer;

    tf::TransformListener tf_;

    nav_msgs::OccupancyGrid local_map;
    vector<int> free_cell;

    Pose root_, goal_;

    double goal[3], root[3];

    rrts *rrtstar;

    bool root_received, goal_received, map_received;

    void init_planner();
    void local_map_Callback(const pnc_msgs::local_map lm );
    Pose local_map_transform(Pose transform_pose);
    void send_rrts_status(const ros::TimerEvent &e);
    void get_sub_goal(Pose sub_goal);
    void get_robot_pose(Pose robot_pose);
    int get_trajectory();
    void comitted_trajectory_pub();
};

#endif /* RRTS_REPLAN_H_ */
