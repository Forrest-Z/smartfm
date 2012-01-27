/*
 * adc_node.cpp
 *
 *  Created on: May 19, 2011
 *      Author: golfcar
 */


#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

void adcCallBack(std_msgs::Float64 adc_value)
{
	btMatrix3x3 btm;// = new btMatrix3x3();
	btm.setRPY(0, (adc_value.data-300.0)/180.0*M_PI, 0);
	tf::Quaternion qt_temp;
	btm.getRotation(qt_temp);
	static tf::TransformBroadcaster broadcaster_b;

	broadcaster_b.sendTransform(
			tf::StampedTransform(
					tf::Transform(qt_temp, tf::Vector3(0, 0, 0)),
					ros::Time::now(),"/tilt_hokuyo", "/tilt_base"));
}
int
main( int    argc,
		char** argv  )
{

	ros::init(argc, argv, "adc_broadcast");
	ros::NodeHandle n;
	ros::Subscriber adc_sub = n.subscribe("adc", 2, adcCallBack);

	ros::spin();
	return 1 ;
}
