/*
 * gplan_scheduler.h
 *
 *  Created on: Jul 22, 2011
 *      Author: golfcar
 */

#ifndef __GOLFCAR_GP__GPLAN_SCHEDULER_H__
#define __GOLFCAR_GP__GPLAN_SCHEDULER_H__



#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Point32.h>

#include <StationPath.h>


namespace golfcar_gp
{

class GlobalPlan : public nav_core::BaseGlobalPlanner {

public:
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    ros::Publisher g_plan_pub_;
    ros::Publisher direction_pub_;
    ros::Subscriber stage_sub_;

    StationPaths sp_;
};

}

#endif /* GPLAN_SCHEDULER_H_ */
