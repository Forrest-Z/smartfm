/*
 * SMSpeedControl.hpp
 *
 *  Created on: Dec 7, 2013
 *      Author: liuwlz
 *
 *      Reference from dynamic_safety_zone by golfcart
 *
 */

#ifndef SMSPEEDCONTROL_HPP_
#define SMSPEEDCONTROL_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PolygonStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/UInt16.h>
#include <pnc_msgs/move_status.h>
#include <fmutil/fm_filter.h>
#include <boost/bind.hpp>

#include <stdlib.h>
#include <string>

using namespace std;

namespace MPAV{

	class SMSpeedControl{

		typedef sensor_msgs::PointCloud pc_t;
		typedef sensor_msgs::LaserScan laser_t;
		typedef nav_msgs::Odometry odom_t;
		typedef	 geometry_msgs::Twist twist_t;
		typedef	 geometry_msgs::PolygonStamped poly_t;

		ros::NodeHandle nh;

	    tf::TransformListener tf_;
	    tf::MessageFilter<laser_t> * tf_filter_;
	    message_filters::Subscriber<laser_t> laser_sub_;
	    laser_geometry::LaserProjection projector_;

	    ros::Subscriber odom_sub_, advised_vel_sub_, steer_angle_sub_;
	    ros::Publisher processed_vel_pub_, dead_zone_pub_, dynamic_zone_pub_, obst_view_pub_, song_status_pub;

	    twist_t advised_vel, processed_vel;
	    pc_t obst_pc, laser_pc;
	    poly_t danger_zone;

	    fmutil::LowPassFilter vFilter_;
	    ros::Time filter_time;

	    typedef enum{Normal, SlowMove, Replan, TempStop} SPEEDSTATUS;
	    SPEEDSTATUS SpeedState;

	    std_msgs::UInt16 SongStatus;
	    ros::Time stop_time_;
	    bool stop_time_init_;

	    double stop_time_threshold;
	    double offcenter_x, offcenter_y;
	    double x_buffer, y_buffer;
	    double x_coeff, y_coeff, angle_coeff;
	    double current_vel, current_angle;
	    double front_boundary, side_boundary, back_boundary;
	    double vel_inc, replan_vel_inc, temp_stop_dec;
	    double replan_vel, slow_move_ratio;

	public:
	    string base_frame;
	    ros::Timer vel_process_timer_;

		SMSpeedControl();
		~SMSpeedControl();
		void OdomCallBack(const odom_t odomIn);
		void LaserCallBack(const laser_t::ConstPtr &laserIn);
		void AdvisedSpeedCallBack(const twist_t velIn);
		void SteerAngleCallBack(const pnc_msgs::move_status moveStatusIn);

		void GetObstacleStatus();
		void SpeedControlTimer(const ros::TimerEvent &e);
		void GetDynamicSpeed(double &dynamic_speed);
		void NormalPlanSpeedControl();
		void ReplanSpeedControl();
		void SlowMoveSpeedControl();
		void TempStopControl();
		void PedInterfaceVoice();
		void SetStatus(int status);
	};
}



#endif /* SMSPEEDCONTROL_HPP_ */
