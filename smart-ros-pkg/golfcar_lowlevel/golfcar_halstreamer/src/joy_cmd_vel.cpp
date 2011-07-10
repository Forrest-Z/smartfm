#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include "joy/Joy.h"
#include <math.h>

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

ros::Publisher speed_pub;
ros::Publisher steering_pub;
ros::Publisher brake_pub;
void joyCallBack(joy::Joy joy_)
{
	geometry_msgs::Twist tw;
	tw.linear.x=joy_.axes[1]*2.5;
	
	tw.angular.z=joy_.axes[0]*40/180*M_PI;
	//ROS_INFO("Speed=%lf, Steering=%lf, brake=%lf", speed.volt, st.angle, bp.angle);
	
	speed_pub.publish(tw);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "joyhook");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joy", 1000, joyCallBack);
	speed_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	puts("Reading from Joystick");
	puts("---------------------------");
 	
 	ros::spin();
	
	return 0;

}



