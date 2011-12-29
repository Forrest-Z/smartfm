#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "StationPath.h"

using namespace std;


/**
 * Prompts passenger for destination (station number), and publishes it on
 * "move_base_simple/goal" as geometry_msgs::PoseStamped messages:
 * goal.x is the current station and goal.y is the destination station.
 */


int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_talker");
    ros::NodeHandle nh;
    ros::Publisher waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);

    geometry_msgs::PoseStamped goal;
    StationList stations;

    stations.print();
    Station s = stations.prompt("Current station ?");
    cout <<"Current station is " <<s.str() <<endl;
    goal.pose.position.x = (double) s.number();

    while( ros::ok() )
    {
        stations.print();
        s = stations.prompt("Next station ?");
        cout <<"Next station is " <<s.str() <<endl;
        
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "/map";
        
        goal.pose.position.y = (double) s.number();
        goal.pose.orientation.z = 1.0;
        waypoint_pub_.publish(goal);
        
        ROS_INFO("sent goal: [%f %f]", goal.pose.position.x, goal.pose.position.y);
        goal.pose.position.x = goal.pose.position.y;
    }

    return 0;
}
