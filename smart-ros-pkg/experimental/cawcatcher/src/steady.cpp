 #define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <math.h>
#include <cawcatcher/CalPotRead.h>
#include <cawcatcher/AngleComm.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

cawcatcher::AngleComm angle_msg;
cawcatcher::CalPotRead attitude;

ros::Publisher *angle_pub;
ros::Publisher *attitude_pub;
    
int main(int argc, char **argv)
{

    ros::init(argc, argv, "steady");
    ros::NodeHandle n;
    ros::Rate  r(10);


    angle_msg.LRoll_com = 0;
    angle_msg.LPitch_com = 0;
    angle_msg.RRoll_com = 0;
    angle_msg.RPitch_com = 0;

    attitude.Lroll = 0;
    attitude.Lpitch = 0;
    attitude.Rroll = 0;
    attitude.Rpitch = 0;

    angle_pub = new ros::Publisher(n.advertise<cawcatcher::AngleComm>("cawcatcherIN_2", 1));
    attitude_pub = new ros::Publisher(n.advertise<cawcatcher::CalPotRead>("attitude", 1));

    while (ros::ok())
	{
    angle_msg.header.stamp = ros::Time::now();
    attitude.header.stamp = ros::Time::now();

    angle_pub->publish(angle_msg);
    attitude_pub->publish(attitude);

    ros::spinOnce();
	 r.sleep();
	}
    return 0;
}

