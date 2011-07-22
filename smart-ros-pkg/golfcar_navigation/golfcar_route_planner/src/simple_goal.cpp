#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
ros::Publisher waypoint_pub_;
void goalCallback(geometry_msgs::PoseStamped map_pose)
{
	geometry_msgs::PoseStamped odom_pose;
	tf::TransformListener tf_;
	try {
		tf_.transformPose("/odom", map_pose, odom_pose);
	}
	catch(tf::LookupException& ex) {
		ROS_ERROR("No Transform available Error: %s\n", ex.what());
	}
	catch(tf::ConnectivityException& ex) {
		ROS_ERROR("Connectivity Error: %s\n", ex.what());
	}
	catch(tf::ExtrapolationException& ex) {
		ROS_ERROR("Extrapolation Error: %s\n", ex.what());
	}
	geometry_msgs::PointStamped odom_point;
	odom_point.header.stamp = ros::Time::now();
	odom_point.header.frame_id = "/odom";
	odom_point.point.x = odom_pose.pose.position.x;
	odom_point.point.y = odom_pose.pose.position.y;
	odom_point.point.z = tf::getYaw(odom_pose.pose.orientation);
	waypoint_pub_.publish(odom_point);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("move_base_simple/goal",1,goalCallback);
	waypoint_pub_ = n.advertise<geometry_msgs::PointStamped>("pnc_waypoint", 100);
	ros::spin();
	return 0;
}
