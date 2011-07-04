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
	
	int pose_init;
	double pose_pre;
	double pose_zero;
	double yaw_pre;
	double time_pre;
	double sampling_rate;
	nav_msgs::Odometry odom;
	
	double yaw;
	
	double speed_pre_;

	golfcar_odometry_imu(ros::NodeHandle nh_);

	~golfcar_odometry_imu();

	void pose2DCallBack(geometry_msgs::Pose2D pose2d);
	void samplerCallBack(golfcar_odom::odo sampler);
	void imuCallBack(sensor_msgs::Imu imu);
	bool imu_started;
	geometry_msgs::Quaternion imuQ;
	double pose_y;
	double laser_yaw;
	double yaw_rate;
	double yaw_;

private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Subscriber lodo;
	ros::Subscriber imu;
};
};
