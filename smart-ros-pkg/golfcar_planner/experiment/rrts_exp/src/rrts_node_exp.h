/*
 * rrts_node.h
 *
 *  Created on: Apr 24, 2013
 *      Author: liuwlz
 */

#ifndef RRTS_NODE_EXP_H_
#define RRTS_NODE_EXP_H_

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

#include <message_filters/subscriber.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pnc_msgs/local_map.h>
#include <rrts/rrts_status.h>

#include "dubins_car_exp.h"
#include "rrts_exp.hpp"

using namespace std;

typedef DubinsCarExp::StateTypeExp state_t;
typedef DubinsCarExp::TrajectoryTypeExp trajectory_t;
typedef DubinsCarExp::SystemTypeExp system_t;

typedef RRTstarExp::VertexExp <DubinsCarExp> vertex_t;
typedef RRTstarExp::RRTSPlanner <DubinsCarExp> planner_t;

class PlannerExp
{
  public:
    PlannerExp();
    ~PlannerExp();

    system_t system;
    planner_t rrts;
    double planner_dt;

    geometry_msgs::Point32 goal;
    nav_msgs::OccupancyGrid map;
    bool is_first_goal, is_first_map;

    geometry_msgs::Point32 car_position;
    tf::TransformListener tf_;
    int get_robot_pose();

    double max_length_committed_trajectory;
    bool is_updating_committed_trajectory;
    void publish_committed_trajectory();
    list<double*> committed_trajectory;
    list<float> committed_control;
    void publish_control_view_trajectory();

    int clear_committed_trajectory();
    int clear_committed_trajectory_length();
    bool should_send_new_committed_trajectory;
    bool is_first_committed_trajectory;
    bool is_near_end_committed_trajectory();
    double state_last_clear[3];

    // ros
    ros::NodeHandle nh;

    ros::Subscriber goal_sub;
    ros::Subscriber map_sub;

#define NUM_STATUS  (7)
    enum status_def{rinc=0, ginc, ginf, ring, rnr, swr,trjf};
    bool rrts_status[NUM_STATUS];

    ros::Publisher rrts_status_pub, obs_check_pub;
    ros::Timer rrts_status_timer, obs_check_timer;

    ros::Publisher tree_pub;
    ros::Publisher vertex_pub;
    ros::Publisher control_trajectory_pub;

    ros::Publisher committed_trajectory_pub;
    ros::Publisher committed_trajectory_view_pub;
    ros::Publisher sampling_view_pub;
    ros::Timer planner_timer;
    ros::Timer tree_pub_timer;
    ros::Timer committed_trajectory_pub_timer;

    // functions
    void on_goal(const geometry_msgs::PoseStamped::ConstPtr p);
    void send_rrts_status(const ros::TimerEvent &e);
    void on_map(const pnc_msgs::local_map::ConstPtr lm);
    void on_committed_trajectory_pub_timer(const ros::TimerEvent &e);
    void obs_check();

    bool root_in_goal();
    bool is_robot_in_collision();
    void change_goal_region();
    void setup_rrts();
    void on_planner_timer(const ros::TimerEvent &e);
    int get_plan();
    float dist(float x1, float y1, float z1, float x2, float y2, float z2)
    {
      return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
    }
    void publish_tree();
    void on_tree_pub_timer(const ros::TimerEvent &e);
};



#endif /* RRTS_NODE_H_ */
