#include <ros/ros.h>
#include <std_msgs/Float32.h>

ros::Publisher * pan_angle_pub_;
ros::Publisher * tilt_angle_pub_;

void panAngleCB(const std_msgs::Float32::ConstPtr pan_angle_ptr)
{
	ROS_INFO("call back!");
	std_msgs::Float32 pan_angle;
	pan_angle.data = pan_angle_ptr->data;
	pan_angle_pub_->publish(pan_angle);
}

void tiltAngleCB(const std_msgs::Float32::ConstPtr tilt_angle_ptr)
{
	std_msgs::Float32 tilt_angle;
	tilt_angle.data = tilt_angle_ptr->data;
	tilt_angle_pub_->publish(tilt_angle);
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"pan_tilt_udp_remap");
	ros::NodeHandle nh;
	ros::Subscriber pan_angle_sub = nh.subscribe("pan_servo_angle_udp",1,panAngleCB,ros::TransportHints().udp());
	ros::Subscriber tilt_angle_sub = nh.subscribe("tilt_servo_angle_udp",1,tiltAngleCB,ros::TransportHints().udp());

	ros::Publisher pan_angle_pub = nh.advertise<std_msgs::Float32>("pan_servo_angle",1);
	ros::Publisher tilt_angle_pub = nh.advertise<std_msgs::Float32>("tilt_servo_angle",1);

	pan_angle_pub_ = & pan_angle_pub;
	tilt_angle_pub_ = & tilt_angle_pub;

	ros::spin();

	return 0;
}
