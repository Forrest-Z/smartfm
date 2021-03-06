/*
 * odom_baselink.cpp
 *
 *  Created on: Nov 30, 2012
 *      Author: demian
 */

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

using namespace std;

string odom_frame_id_, baselink_frame_id_;
void odomCallback(nav_msgs::OdometryConstPtr msg)
{
	static tf::TransformBroadcaster broadcaster_b;

	btScalar pitch, roll, yaw;
	geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
	btQuaternion btq(orientation.x, orientation.y, orientation.z, orientation.w);
	btMatrix3x3(btq).getEulerYPR(yaw, pitch, roll);

	//correct the roll angle from +-180 to +-0
	//if(roll>=0) roll-=M_PI;
	//else roll+=M_PI;

	btMatrix3x3 btm;
	btm.setEulerYPR(yaw, 0, 0);
	btQuaternion btqt_temp;
	btm.getRotation(btqt_temp);

	broadcaster_b.sendTransform(
			tf::StampedTransform(
					tf::Transform(btqt_temp, tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,0)),
					msg->header.stamp, odom_frame_id_, baselink_frame_id_));


}

int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "odom_baselink");
	ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    priv_nh.param("odom_frame_id", odom_frame_id_, string("odom"));
    priv_nh.param("baselink_frame_id", baselink_frame_id_, string("odom_baselink"));
	ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);
	ros::spin();
}
