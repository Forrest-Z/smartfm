/*
 * time_server.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: golfcar
 */

#include <sys/time.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

int main( int argc, char *argv[] )
{
    ros::init(argc, argv, "time_server");
    ros::NodeHandle nh;
    struct timeval time;

    ros::Publisher pub = nh.advertise<rosgraph_msgs::Clock>("clock", 1);
    while(ros::ok())
    {
        gettimeofday( &time, NULL );
        ros::Time ros_time;
        ros_time.nsec = time.tv_usec*1000;
        ros_time.sec = time.tv_sec;
        rosgraph_msgs::Clock time_msg;
        time_msg.clock = ros_time;
        pub.publish(time_msg);
    }
}
