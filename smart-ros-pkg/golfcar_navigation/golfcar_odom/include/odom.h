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
#include <golfcar_odom/odo.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include "math.h"

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

namespace golfcar_odometry{
class golfcar_odometry
{
public:

        ros::Publisher odom_pub;
        ros::Publisher filter_pub;
        int pose_init;
        double pose_pre;
        double pose_zero;
        double yaw_rate_pre;
        double time_pre;
        double sampling_rate;
        nav_msgs::Odometry odom;
        golfcar_odom::odo golfcarodo;
        double yaw;
        std::vector<double> speed_;
        std::vector<double> acc_;
        double speed_pre_;

        golfcar_odometry(ros::NodeHandle nh_);
        
        ~golfcar_odometry();

        void pose2DCallBack(geometry_msgs::Pose2D pose2d);
        void samplerCallBack(golfcar_odom::odo sampler);
        
		void imuCallBack(sensor_msgs::Imu imu);
        geometry_msgs::Quaternion imuQ;
        
private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber lodo;
};
};
