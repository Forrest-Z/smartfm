#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
geometry_msgs::PoseStamped goal;
int main(int argc, char **argv)
{
	ros::init(argc,argv,"goal_publisher");
	ros::NodeHandle nh;
	ros::Duration(2.0).sleep();
	ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);

	while(ros::ok()){
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "/map";
	goal.pose.position.x =  0;
	goal.pose.position.y = 2;
	goal.pose.orientation.z = 1.0;
	goal_pub.publish(goal);

	cout<<"goal pubed"<<endl;
	goal_pub.publish(goal);
	
	cout<<"goal pubed"<<endl;
	goal_pub.publish(goal);

	cout<<"goal pubed"<<endl;
	ros::spinOnce();
	ros::Duration(0.1).sleep();
	}
}


