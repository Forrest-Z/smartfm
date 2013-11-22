#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"publishFakeOdom");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	double velocity;
	private_nh.param("velocity",velocity,2.0);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("can_imu_odom",10);
	ros::Rate r(10);
	while(ros::ok())
	{
	nav_msgs::Odometry odom;
	odom.header.seq = 0;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";
	odom.twist.twist.linear.x = velocity;
	odom_pub.publish(odom);
	r.sleep();
	}
}
