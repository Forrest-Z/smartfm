#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <nav_core/base_global_planner.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>


using namespace costmap_2d;




namespace golfcar_gp
{

class SimpleGlobalPlan : public nav_core::BaseGlobalPlanner
{
public:
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    ros::Publisher g_plan_pub_;
    ros::Publisher direction_pub_;
    ros::Subscriber stage_sub_;

    bool firstpart_;
    bool part1a_;
    bool firstpart;
    int stage_;

    void missionStage(geometry_msgs::Point32 stage);
};


void SimpleGlobalPlan::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ros::NodeHandle global_node;
    g_plan_pub_ = global_node.advertise<nav_msgs::Path>("global_plan", 1);
}

bool SimpleGlobalPlan::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan)
{
    plan.push_back(start);
    plan.push_back(goal);

    nav_msgs::Path p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "/map";
    p.poses.resize(2);
    p.poses[0] = start;
    p.poses[1] = goal;
    g_plan_pub_.publish(p);

    return true;
}

}


PLUGINLIB_DECLARE_CLASS(golfcar_gp, SimpleGlobalPlan, golfcar_gp::SimpleGlobalPlan, nav_core::BaseGlobalPlanner)
