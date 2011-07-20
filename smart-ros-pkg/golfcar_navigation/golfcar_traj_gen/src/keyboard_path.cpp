#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sstream>
#include <tf/tf.h>
#include "math.h"

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

int main(int argc, char **argv)
{

	ros::init(argc, argv, "keyboard_path");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<nav_msgs::Path>("pnc_trajectory",1000);
	signal(SIGINT,quit);
	char c;

	

	//turning radius in meter
	float radius = 3;

	//min number of points
	float resolution = 28;

	//seperation of 8 figure
	float seperation = 1.75;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use Left arrow to give a circle, Right arrow for a path in 8");


	for(;;)
	{
		// get the next event from the keyboard
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}
		nav_msgs::Path path;
		path.header.stamp = ros::Time::now();
		path.header.frame_id = "/odom";

		double posex_pre(0), posey_pre(0);
		double posex_cur(0), posey_cur(0);
		switch(c)
		{
		case KEYCODE_L:
		{
			path.poses.resize(resolution+1);
			for(int i=0;i<resolution+1;i++)
			{
				path.poses[i].header.seq = i;
				path.poses[i].header.stamp = ros::Time::now();
				path.poses[i].header.frame_id = "/odom";
				posex_cur = radius*sin(i/resolution*2.0*M_PI) ;
				posey_cur = radius*cos(i/resolution*2.0*M_PI)-radius;
				path.poses[i].pose.position.x = posex_cur;
				path.poses[i].pose.position.y = posey_cur;
				path.poses[i].pose.position.z = atan2(posey_cur-posey_pre, posex_cur-posex_pre);
				path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(atan2(posey_cur-posey_pre, posex_cur-posex_pre));
				posex_pre = posex_cur;
				posey_pre = posey_cur;
			}
			ROS_INFO("Trajectory of a circle with radius %lf meter sent", radius);
		}
		break;
		case KEYCODE_R:
		{
			path.poses.resize(resolution+4);

			for(int i=0;i<=resolution/2;i++)
			{
				path.poses[i].pose.position.x = radius*sin(i/resolution*2.0*M_PI) ;
				path.poses[i].pose.position.y = radius*cos(i/resolution*2.0*M_PI)-radius;
			}
			path.poses[resolution/2+1].pose.position.x = -seperation*radius;
			path.poses[resolution/2+1].pose.position.y = 0;
			for(int i=resolution-1;i>resolution/2-2;i--)
			{
				path.poses[resolution-i+resolution/2+1].pose.position.x = radius*sin(i/resolution*2.0*M_PI)-seperation*radius ;
				path.poses[resolution-i+resolution/2+1].pose.position.y = radius*cos(i/resolution*2.0*M_PI)-radius;
			}
			path.poses[resolution+3].pose.position.x = 0;
			path.poses[resolution+3].pose.position.y = 0;

			for(int i=0; i< path.poses.size(); i++)
			{
				path.poses[i].header.seq = i;
				path.poses[i].header.stamp = ros::Time::now();
				path.poses[i].header.frame_id = "/odom";
				posex_cur = path.poses[i].pose.position.x;
				posey_cur = path.poses[i].pose.position.y;
				path.poses[i].pose.position.z = atan2(posey_cur-posey_pre, posex_cur-posex_pre);
				path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(atan2(posey_cur-posey_pre, posex_cur-posex_pre));
				posex_pre = posex_cur;
				posey_pre = posey_cur;
			}
			path.poses[resolution/2+1].pose.position.z = path.poses[resolution/2].pose.position.z;
			path.poses[resolution+3].pose.position.z = path.poses[resolution+2].pose.position.z;

			ROS_INFO("Trajectory of a 8 with radius %lf meter sent", radius);
		}
		break;
		case KEYCODE_U:
		{
			geometry_msgs::PoseStamped pose;
			for(int i=0;i<=resolution/2;i++)
			{
				
				pose.pose.position.x = radius*sin(i/resolution*2.0*M_PI) ;
				pose.pose.position.y = radius*cos(i/resolution*2.0*M_PI)-radius;
				path.poses.push_back(pose);
			}
			
			for(int i=0;i<(seperation*resolution/2.0/M_PI);i++)
			{
				pose.pose.position.x -= radius * 2.0*M_PI/resolution;
				path.poses.push_back(pose);
			}			

			for(int i=resolution/2+1;i<=resolution;i++)
			{
				pose.pose.position.x = radius*sin(i/resolution*2.0*M_PI) -seperation*radius;
				pose.pose.position.y = radius*cos(i/resolution*2.0*M_PI)-radius;
				path.poses.push_back(pose);
			}

			for(int i=0;i<(seperation*resolution/2.0/M_PI);i++)
			{
				pose.pose.position.x += radius * 2.0*M_PI/resolution;
				path.poses.push_back(pose);
			}			


			for(int i=0; i< path.poses.size(); i++)
			{
				path.poses[i].header.seq = i;
				path.poses[i].header.stamp = ros::Time::now();
				path.poses[i].header.frame_id = "/odom";
				posex_cur = path.poses[i].pose.position.x;
				posey_cur = path.poses[i].pose.position.y;
				path.poses[i].pose.position.z = atan2(posey_cur-posey_pre, posex_cur-posex_pre);
				path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(atan2(posey_cur-posey_pre, posex_cur-posex_pre));
				posex_pre = posex_cur;
				posey_pre = posey_cur;
			}
			ROS_INFO("Trajectory of oval with radius %lf meter sent", radius);
		}
			break;
		case KEYCODE_D:

			break;
		case KEYCODE_Q:

			break;
		}
		pub.publish(path);
	}


	return 0;
}
