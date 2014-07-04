#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
using namespace std;
int main(int argc, char**argv)
{
	ros::init(argc,argv,"goal_publisher");
	ros::NodeHandle nh;
	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("new_global_plan", 100);
	nav_msgs::Path p;
	p.poses.resize(20);
	for(size_t i=0; i<20; i++){
		p.poses[i].pose.position.x = 10+i;
		p.poses[i].pose.position.y = 10+i;
		p.poses[i].pose.orientation.w = 1.0;
	}
	p.header.stamp=ros::Time::now();
	p.header.frame_id="/map";
	while(ros::ok()){
		path_pub.publish(p);
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
}
