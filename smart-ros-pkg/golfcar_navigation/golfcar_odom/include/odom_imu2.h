/*
 * odom.h
 *
 *  Created on: Mar 6, 2011
 *      Author: demian
 */

#ifndef ODOM_H_
#define ODOM_H_


#endif /* ODOM_H_ */
#include <ros/ros.h>
#include "golfcar_odom/odo.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include "math.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
namespace golfcar_odometry_imu{
class golfcar_odometry_imu
{
public:

	ros::Publisher odom_pub;
	ros::Publisher filter_pub;
	int pose_init;
	
	double yaw_pre;
	
	nav_msgs::Odometry odom;
	golfcar_odom::odo golfcarodo;
	

	golfcar_odometry_imu(ros::NodeHandle nh_);

	~golfcar_odometry_imu();

	void samplerCallBack(golfcar_odom::odo sampler);
	void imuCallBack(sensor_msgs::Imu imu);
	bool initialized;
	geometry_msgs::Quaternion imuQ;
	double pose_pre;

private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Subscriber imu;
};
};
