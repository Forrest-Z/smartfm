#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "SimpleGoal.h"
#include <std_msgs/UInt16.h>

using namespace std;


SimpleGoal::SimpleGoal(const StationPaths & sp) : RoutePlanner(sp)
{
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);
    sound_pub_ = nh.advertise<std_msgs::UInt16>("voice_id", 1);
    speed_status_sub_ = nh.subscribe("speed_status", 1, &SimpleGoal::speedStatusCallBack, this);
    has_reached_ = false;
    reachSoundPlayed_ = false;
    world_coord_sub_ = nh.subscribe("/world_utm_latlon", 1, &SimpleGoal::worldCoordCallBack, this);
}

void SimpleGoal::initDest(const Station & start, const Station & end)
{
    has_reached_ = false;
    reachSoundPlayed_ = false;
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "/map";
    goal.pose.position.x = (double) start.number();
    goal.pose.position.y = (double) end.number();
    goal.pose.orientation.z = 1.0;
    goal_pub_.publish(goal);
    std_msgs::UInt16 sound_data;
    sound_data.data = 1;
    sound_pub_.publish(sound_data);

    ROS_INFO("sent goal: [%f %f]", goal.pose.position.x, goal.pose.position.y);
}

void SimpleGoal::speedStatusCallBack(const pnc_msgs::speed_contribute &msg)
{
	goal_pub_.publish(goal);
    has_reached_ = msg.goal;
    eta_ = has_reached_ ? 0 : msg.dist_goal / 2; //velocity is taken as constant 2m/s
}

bool SimpleGoal::goToDest()
{
    if(!reachSoundPlayed_ && has_reached_){
        std_msgs::UInt16 sound_id;
        sound_id.data = 2;
        reachSoundPlayed_ = true;
        sound_pub_.publish(sound_id);
    }
    return has_reached_;
}

void SimpleGoal::worldCoordCallBack(const map_to_world::coordinate & msg)
{
    latitude_ = msg.latitude;
    longitude_ = msg.longitude;
}
