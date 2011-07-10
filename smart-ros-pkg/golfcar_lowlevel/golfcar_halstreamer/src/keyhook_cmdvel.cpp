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
  
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "keyhook");
	ros::NodeHandle n;
	
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	signal(SIGINT,quit);
 

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Type a desired velocity to move the car.");


  for(;;)
  {
	double vel;
    std::cin >> vel; 
	geometry_msgs::Twist tw;
	tw.linear.x=vel;
	//velocity.angle=turning*t_factor;
	ROS_INFO("Speed=%lf", tw.linear.x);
	
	chatter_pub.publish(tw);
  }
	

  return 0;
}
