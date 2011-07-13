#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "joy/Joy.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

ros::Publisher cmd_pub;

void joyCallBack(joy::Joy joy_)
{
	geometry_msgs::Twist cmd;
	if(joy_.axes[1]>0)
	  cmd.linear.x = joy_.axes[1]*3;
	else
	  cmd.linear.x = 0;

	ROS_INFO("speed cmd = %3.2lf", cmd.linear.x);

	cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "joyhook");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joy", 1000, joyCallBack);
	cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	puts("Reading from Joystick");
	puts("---------------------------");
 	
 	ros::spin();
	
	return 0;

}
