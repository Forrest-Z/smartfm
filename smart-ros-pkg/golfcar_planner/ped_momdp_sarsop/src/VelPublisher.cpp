#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
ros::Subscriber vel_sub;
double vel;
ros::Publisher cmd_pub;
void velCallBack(geometry_msgs::TwistConstPtr pomdp_vel)
{
	vel=pomdp_vel->linear.x;
}
void publishSpeed(const ros::TimerEvent& event)
{
	geometry_msgs::Twist cmd;
	cmd.angular.z = 0;       	
	cmd.linear.x = vel;
	cmd_pub.publish(cmd);
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"vel_publisher");
	ros::NodeHandle nh;
	vel_sub=nh.subscribe("cmd_vel_pomdp",1,&velCallBack);
	ros::Timer timer= nh.createTimer(ros::Duration(0.05),&publishSpeed);
	cmd_pub= nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::spin();
	return 0;
}
