/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 08/11/2010
*********************************************************************/
#include "MTi/MTi.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
int main(int argc, char** argv)
{
	ros::init(argc, argv, "mti_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	// Params
	std::string portname;
	int baudrate;
	std::string frame_id;
	pn.param<std::string>("port", portname, "/dev/ttyUSB0");
	pn.param("baudrate", baudrate, 115200);
	pn.param<std::string>("frame_id", frame_id, "/base_imu");
	
	Xsens::MTi * mti = new Xsens::MTi();
	
	if(!mti->openPort((char*)portname.c_str(), baudrate))
	{
		ROS_FATAL("MTi -- Unable to connect to the MTi.");
		ROS_BREAK();
	}
	ROS_INFO("MTi -- Successfully connected to the MTi!");
	
	Xsens::MTi::outputMode outputMode;
	outputMode.temperatureData = false;
	outputMode.calibratedData = true;
	outputMode.orientationData = true;
	outputMode.auxiliaryData = false;
	outputMode.positionData = true;
	outputMode.velocityData = true;
	outputMode.statusData = true;
	outputMode.rawGPSData = false;
	outputMode.rawInertialData = false;
	
	Xsens::MTi::outputSettings outputSettings;
	outputSettings.timeStamp = false;
	outputSettings.orientationMode = Xsens::Quaternion;
	
	if(!mti->setOutputModeAndSettings(outputMode, outputSettings, 1000))
	{
		ROS_FATAL("MTi -- Unable to set the output mode and settings.");
		ROS_BREAK();
	}
	ROS_INFO("MTi -- Setup complete! Initiating data streaming...");

	ros::Publisher mti_pub = n.advertise<sensor_msgs::Imu>("imu/data", 10);
	ros::Publisher mti_pose_pub = n.advertise<geometry_msgs::PoseStamped>("xsens/odom",10);
	ros::Rate r(200);
	
	ros::Publisher odom_pub =  n.advertise<nav_msgs::Odometry>("odom", 100);
	nav_msgs::Odometry odom;
	double odom_x(0), odom_y(0), odom_z(0);
	bool initialize=false;
	double yaw_pre(0);
  	while(ros::ok())
	{	
		sensor_msgs::Imu mti_msg;
		geometry_msgs::PoseStamped mti_pose;
        mti_pose.header.stamp = ros::Time::now();
        mti_pose.header.frame_id = "xsens/odom";

		mti_pose.pose.position.x = mti->pos_x();
		mti_pose.pose.position.y = mti->pos_y();
		mti_pose.pose.position.z = mti->pos_z();
		
		mti_pose.pose.orientation.x = mti->vel_x();
		mti_pose.pose.orientation.y = mti->vel_y();
		mti_pose.pose.orientation.z = mti->vel_z();
		mti_pose.pose.orientation.w = mti->status();
		
		mti_msg.header.stamp = ros::Time::now();
		mti_msg.header.frame_id = frame_id.c_str();
		
		mti_msg.orientation.x = mti->quaternion_x();
		mti_msg.orientation.y = mti->quaternion_y();
		mti_msg.orientation.z = mti->quaternion_z();
		mti_msg.orientation.w = mti->quaternion_w();
		
		
		mti_msg.angular_velocity.x = mti->gyroscope_x();
		mti_msg.angular_velocity.y = mti->gyroscope_y();
		mti_msg.angular_velocity.z = mti->gyroscope_z();
		
		mti_msg.linear_acceleration.x = mti->accelerometer_x();
		mti_msg.linear_acceleration.y = mti->accelerometer_y();
		mti_msg.linear_acceleration.z = mti->accelerometer_z();
		
		static tf::TransformBroadcaster broadcaster_b;
		tf::Quaternion qt_temp;
		btScalar pitch, roll, yaw;
		tf::Quaternion qt;
		geometry_msgs::Quaternion orientation = mti_msg.orientation;
		btQuaternion btq(orientation.x, orientation.y, orientation.z, orientation.w);
		btMatrix3x3(btq).getEulerYPR(yaw, pitch, roll);
		
		//handle the nan issue. This occur when the imu is restarted 
		if(isnan(yaw)||isnan(pitch)||isnan(roll)) 
		{
			odom_x=odom_y=odom_z=0;
			continue;
		}
		
		btMatrix3x3 btm;
		pitch = -pitch;
		yaw = -yaw;
		
		if(!initialize) 
		{
			yaw_pre = yaw;
			initialize = true;
		}
		//Ensure that the orientation always start from yaw=0. That's the assumption made for odometry calculation
		yaw -= yaw_pre;
		btm.setEulerYPR(yaw, pitch, roll);
		btQuaternion btqt_temp;
		btm.getRotation(btqt_temp);
		tf::quaternionTFToMsg(btqt_temp,mti_msg.orientation);
		//test odometry, assuming running at 1 m/s
		
		double r11 = cos(yaw)*cos(pitch);
		double r21 = sin(yaw)*cos(pitch);
		double r31 = -sin(-pitch);
		double distance = 0.005;
		odom_x = distance * r11;
		odom_y = distance * r21;
		odom_z = distance * r31;
		
		
		odom.header.frame_id = "odom";
		odom.header.stamp = ros::Time::now();
		
		odom.pose.pose.position.x+=odom_x;
		odom.pose.pose.position.y+=odom_y;
		odom.pose.pose.position.z-=odom_z;
		odom.pose.pose.orientation = mti_msg.orientation;
		
		broadcaster_b.sendTransform(
			tf::StampedTransform(
				tf::Transform(qt_temp,tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)),
				ros::Time::now(),"odom", "base_link"));
		
		mti_pub.publish(mti_msg);
		mti_pose_pub.publish(mti_pose);
		odom_pub.publish(odom);
		
		r.sleep();
	}
	
  	return(0);
}

// EOF

