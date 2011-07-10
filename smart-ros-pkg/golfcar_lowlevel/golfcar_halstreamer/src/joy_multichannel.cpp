#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include <ros/ros.h>
#include "golfcar_halstreamer/throttle.h"
#include "golfcar_halstreamer/steering.h"
#include "golfcar_halstreamer/brakepedal.h"
#include <sstream>
#include "joy/Joy.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

int kfd = 0;
struct termios cooked, raw;
double steering_adjust=0.7;
double speed_adjust = 0.2;
double brake_adjust=1;
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
		
	if(joy_.buttons[4]) steering_adjust +=0.1;
	if(joy_.buttons[3]) steering_adjust -=0.1;
	if(steering_adjust >1) steering_adjust=1;
	if(steering_adjust <0) steering_adjust =0;

	if(joy_.buttons[2]) speed_adjust +=0.1;
	if(joy_.buttons[1]) speed_adjust -=0.1;
	if(speed_adjust >1) speed_adjust=1;
	if(speed_adjust <0) speed_adjust =0;
	
	if(joy_.buttons[9]) brake_adjust -=.1;
	if(joy_.buttons[10]) brake_adjust +=.1;
	if(brake_adjust >1) brake_adjust=1;
	if(brake_adjust <0) brake_adjust =0;
	
	golfcar_halstreamer::throttle speed;
	golfcar_halstreamer::steering st;
	golfcar_halstreamer::brakepedal bp;
	if(joy_.axes[1]>0)
		speed.volt=joy_.axes[1]*3.33*speed_adjust;
	else
		bp.angle = -joy_.axes[1]*106*brake_adjust;
	st.angle=-joy_.axes[0]*540*steering_adjust;
	ROS_INFO("Speed=%3.2lf, Steering=%3.2lf, brake=%3.2lf", speed.volt, st.angle, bp.angle);
	ROS_INFO("SpeedA=%1.1lf, SteeringA=%1.1lf, brakeA=%1.1lf", speed_adjust, steering_adjust, brake_adjust);
	
	speed_pub.publish(speed);
	steering_pub.publish(st);
	brake_pub.publish(bp);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "joyhook");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joy", 1000, joyCallBack);
	speed_pub = n.advertise<golfcar_halstreamer::throttle>("golfcar_speed", 1);
	steering_pub = n.advertise<golfcar_halstreamer::steering>("golfcar_steering", 1);
	brake_pub = n.advertise<golfcar_halstreamer::brakepedal>("golfcar_brake", 1);
	puts("Reading from Joystick");
	puts("---------------------------");
 	
 	ros::spin();
	
	return 0;

}



