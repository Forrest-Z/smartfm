/*
 * hybrid_planner.h
 *
 *  Created on: Apr 1, 2013
 *      Author: liuwlz
 */

#ifndef HYBRID_PLANNER_H_
#define HYBRID_PLANNER_H_

#include <math.h>

#include <string>
#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
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

/// Drives the vehicle from A to B by feeding it a sequence of waypoints.
class HybridPlanner
{
public:
    HybridPlanner(const int start, const int end);

private:
    ros::NodeHandle n;
    ros::Publisher g_plan_pub_;
    ros::Publisher nextpose_pub_;
    ros::Publisher norminal_lane_pub_;
    ros::Publisher hybrid_path_pub_;
    ros::Publisher move_status_pub_;

    ros::Subscriber goal_in_collision_sub_;
    ros::Subscriber move_status_sub_;
    ros::Subscriber rrts_path_sub_;

    ros::CallbackQueue rrts_callback;

    ros::Timer stateCheckerTimer;

    tf::TransformListener tf_;
    tf::Stamped<tf::Pose> global_pose_;
    StationPaths sp_;

    nav_msgs::Path global_path;
    nav_msgs::Path local_path;
    nav_msgs::Path hybrid_path;

    StationPath path_;
    Station destination_;
    bool goal_collision_;
    bool root_in_goal_;
    bool robot_near_root_;
    bool switched_root_;
    bool goal_infeasible_;
    bool trajectory_found_;

    pnc_msgs::move_status move_status_;

    bool is_first_goal;
    bool initialized_;
    bool rrts_is_replaning_;
    bool is_first_replan;
    bool is_replan_end;

    geometry_msgs::PoseStamped map_pose;

    unsigned waypointNo_;

    bool goToDest();
    void initDest(const int start, const int end);

    bool getRobotGlobalPose();
    double distanceToGoal();
    void getGlobalPlan();
    void getNearestWaypoints();
    void rrts_status(rrts_exp::rrts_status rrts_status);
    void rrtspathCallBack(const nav_msgs::Path rrts_path);
    void movestatusCallBack(pnc_msgs::move_status move_status);
    void plannerReasonning();
    void rrtsReplanning();

    PlannerExp *planner;
};

#endif /* HYBRID_PLANNER_H_ */
