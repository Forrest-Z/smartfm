#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "SimpleGoal.h"

using namespace std;


SimpleGoal::SimpleGoal(const StationPaths & sp) : RoutePlanner(sp)
{
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);
    speed_status_sub_ = nh.subscribe("speed_status", 1, &SimpleGoal::speedStatusCallBack, this);
    has_reached = false;
}

void SimpleGoal::initDest()
{
    has_reached = false;

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "/map";
    goal.pose.position.x = (double) currentStation_.number();
    goal.pose.position.y = (double) destination_.number();
    goal.pose.orientation.z = 1.0;
    goal_pub_.publish(goal);

    ROS_INFO("sent goal: [%f %f]", goal.pose.position.x, goal.pose.position.y);
}

void SimpleGoal::speedStatusCallBack(const golfcar_ppc::speed_contribute &msg)
{
    has_reached = msg.goal;
}

bool SimpleGoal::goToDest()
{
    return has_reached;
}
