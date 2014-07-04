#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
geometry_msgs::Twist cmd_vel;
ros::Subscriber vel_sub, steer_sub;
ros::Publisher cmd_pub;
void velCallBack(geometry_msgs::TwistConstPtr vel)
{
	cmd_vel.linear.x=vel->linear.x;
}

void steerCallBack(geometry_msgs::TwistConstPtr steer)
{
	cmd_vel.angular.z=steer->angular.z;
}
void publishSpeed(const ros::TimerEvent& event) {
	cmd_pub.publish(cmd_vel); 
}
int main(int argc, char**argv)
{
	ros::init(argc,argv,"vel_mixer");
	ros::NodeHandle nh;
    vel_sub = nh.subscribe("cmd_speed", 1, velCallBack);
	steer_sub=nh.subscribe("cmd_steer", 1, steerCallBack);
	cmd_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), publishSpeed);
	ros::spin();
	return 0;
}
