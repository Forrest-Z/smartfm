/*
 * demo_rrts.h
 *
 *  Created on: Oct 14, 2013
 *      Author: liuwlz
 */

#ifndef DEMO_RRTS_H_
#define DEMO_RRTS_H_

#include <math.h>

#include <string>
#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <svg_path/StationPath.h>
#include <fmutil/fm_math.h>
#include <std_msgs/Bool.h>
#include <rrts_exp/rrts_status.h>
#include <pnc_msgs/move_status.h>
#include <rrts_node_exp.h>

using namespace std;

class RePlanner{

public:
    RePlanner();
    ~RePlanner();

private:
    ros::NodeHandle n;
    ros::Publisher sub_goal_pub_;
    ros::Publisher hybrid_path_pub_;
    ros::Publisher move_status_pub_;

    ros::Subscriber rrts_status_sub_;
    ros::Subscriber move_status_sub_;
    ros::Subscriber rrts_path_sub_;
    ros::Subscriber global_plan_sub_;
    ros::Subscriber obst_pts_sub_;

    tf::TransformListener tf_;
    tf::Stamped<tf::Pose> global_pose_;

    nav_msgs::Path global_path;
    nav_msgs::Path local_path;
    nav_msgs::Path hybrid_path;
    nav_msgs::Path empty_path;

    bool goal_collision_;
    bool root_in_goal_;
    bool robot_near_root_;
    bool switched_root_;
    bool goal_infeasible_;
    bool trajectory_found_;
    bool global_path_found_;

    pnc_msgs::move_status move_status_;

    bool initialized_;
    bool is_first_replan;
    bool is_first_goal;
    bool rrts_is_replaning_;

    geometry_msgs::PoseStamped sub_goal;

    sensor_msgs::PointCloud obst_interest;

    unsigned waypointNo_;

    bool goToDest();
    void getNearestWaypoints();
    bool getRobotGlobalPose();
    void rrtsstatusCallBack(rrts_exp::rrts_status rrts_status);
    void globalPathCallBack(const nav_msgs::Path global_plan);
    void rrtsPathCallBack(const nav_msgs::Path rrts_path);
    void movestatusCallBack(pnc_msgs::move_status move_status);
    void obstCallBack(const sensor_msgs::PointCloud obst_pts);
    void plannerReasonning();
    void rrtsReplanning();
    void replanTimer(const ros::TimerEvent &e);

    PlannerExp *planner;
};

#endif /* DEMO_RRTS_H_ */
