/*
 * keyboard_svg_path.cpp
 *
 *  Created on: Aug 7, 2012
 *      Author: demian
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <StationPath.h>

#include <nav_msgs/Path.h>

using namespace std;

int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "keyboard_svg_path");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<nav_msgs::Path>("pnc_trajectory",1000);
	StationPaths sp_;
	StationPath station_path = sp_.getPath(sp_.knownStations()(atoi(argcv[1])),sp_.knownStations()(atoi(argcv[2])));
	nav_msgs::Path p;
	p.header.stamp = ros::Time::now();
	p.header.frame_id = "/odom";
	p.poses.resize(station_path.size());
	cout<<"Path segments="<<p.poses.size()<<endl;
	cout<<"sp segments="<<station_path.size()<<endl;
	for(unsigned int i=0; i<station_path.size(); i++)
	{
		p.poses[i].header.frame_id = "/odom";
		p.poses[i].header.stamp = ros::Time::now();
		p.poses[i].pose.position.x = station_path[i].x_;
		p.poses[i].pose.position.y = station_path[i].y_;
		p.poses[i].pose.orientation.w = 1.0;
	}
	ros::Rate loop_rate(10);
	int count=0;
	while(ros::ok()&& count++<2)
	{
		p.header.stamp = ros::Time::now();
		for(unsigned int i=0; i<station_path.size(); i++)
			p.poses[i].header.stamp = ros::Time::now();
		pub.publish(p);
		ros::spinOnce();
		loop_rate.sleep();
	}

}
