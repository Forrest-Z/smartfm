#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include <ros/ros.h>
#include "joy/Joy.h"
#include "golfcar_halstreamer/vel.h"
#include <sstream>


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

ros::Publisher chatter_pub;
void joyCallBack(joy::Joy joy_)
{
	golfcar_halstreamer::vel velocity;
	velocity.speed=joy_.axes[1]*joy_.axes[2]*1.5;
	velocity.angle=-joy_.axes[0]*540.0;
	ROS_INFO("Speed=%lf, Angle=%d", velocity.speed, (int)velocity.angle);
	
	chatter_pub.publish(velocity);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "keyhook");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joy", 1000, joyCallBack);
	chatter_pub = n.advertise<golfcar_halstreamer::vel>("golfcar_vel", 1000);
  

	puts("Reading from Joystick");
	puts("---------------------------");
 	
 	ros::spin();
	
	return 0;

}



