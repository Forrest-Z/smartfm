/*
 * ccrrts_node.h
 *
 *  Created on: Aug 3, 2013
 *      Author: liuwlz
 */

#ifndef CCRRTS_NODE_H_
#define CCRRTS_NODE_H_

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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

#include <pnc_msgs/local_map.h>
#include <rrts/rrts_status.h>

#include <dynamic_reconfigure/server.h>
#include "ccrrts/ccrrtsCTRConfig.h"
#include <ccrrts/rrts_result.h>

#include "cc_system.h"
#include "ccrrts.hpp"

using namespace std;

typedef CCDUBINS::CCStateType state_t;
typedef CCDUBINS::CCTrajectoryType trajectory_t;
typedef CCDUBINS::CCSystemType system_t;

typedef CCRRTSNS::CCVertex <CCDUBINS> vertex_t;
typedef CCRRTSNS::CCRRTS <CCDUBINS> planner_t;

class Planner{
  public:
	Planner();
    ~Planner();

    system_t system;
    planner_t ccrrts;

    Level_OF_Risk perform_criteria;
    Level_OF_Risk optimal_criteria;

    double planner_dt, gamma, GoalSampleFreq, risk_limit;
	dynamic_reconfigure::Server<ccrrts::ccrrtsCTRConfig>  *srv;

    geometry_msgs::Point32 goal;
    nav_msgs::OccupancyGrid map;
    bool is_first_goal, is_first_map;

    geometry_msgs::Point32 car_position;
    tf::TransformListener tf_;
    int get_robot_pose();

    void ParamReconfig(ccrrts::ccrrtsCTRConfig &config, uint32_t level);

    double max_length_committed_trajectory;
    bool should_send_new_committed_trajectory;
    bool is_first_committed_trajectory;
    bool is_updating_rrt_tree ;
    bool is_updating_committed_trajectory;
    bool is_system_setup;

    list<double*> committed_trajectory;
    list<float> committed_control;

    void publish_committed_trajectory();
    void publish_vertex_conv(state_t stateIn,visualization_msgs::MarkerArray&vertex_markers, int id);
    void publish_tree();

    int clear_committed_trajectory();
    int clear_committed_trajectory_length();

    bool is_near_end_committed_trajectory();

    double state_last_clear[3];

    // ros
    ros::NodeHandle nh;
    ros::Subscriber goal_sub;
    ros::Subscriber map_sub;

#define NUM_STATUS  (7)
    enum status_def{rinc=0, ginc, ginf, ring, rnr, swr,trjf};
    bool rrts_status[NUM_STATUS];

    ros::Publisher obs_check_pub;

    ros::Publisher tree_pub;
    ros::Publisher vertex_pub;
    ros::Publisher committed_trajectory_view_pub;
    ros::Publisher sampling_view_pub;
    ros::Publisher vertex_marker_pub;
    ros::Publisher planning_result_pub;

    ros::Timer planner_timer;
    ccrrts::rrts_result result_;

    uint32_t cylinder;

    sensor_msgs::PointCloud tree, vertex;
    visualization_msgs::MarkerArray vertex_markers;

    // functions
    void on_goal(const geometry_msgs::PoseStamped::ConstPtr p);
    void on_map(const pnc_msgs::local_map::ConstPtr lm);

    void on_planner_timer(const ros::TimerEvent &e);
    void send_rrts_status(const ros::TimerEvent &e);

    bool root_in_goal();
    bool is_robot_in_collision();
    void change_goal_region();
    void setup_rrts();
    int get_plan();
    double dist(double x1, double y1, double z1, double x2, double y2, double z2){
      return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
    }

};
#endif /* CCRRTS_NODE_H_ */
