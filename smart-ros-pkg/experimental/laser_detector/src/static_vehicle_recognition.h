/*
 * static_vehicle_recognition.h
 *
 *  Created on: Jun 5, 2013
 *      Author: Shen Xiaotong
 */

#ifndef STATIC_VEHICLE_RECOGNITION_H_
#define STATIC_VEHICLE_RECOGNITION_H_

#include <ros/ros.h>

class static_vehicle_recognition{
	ros::NodeHandle nh_;
	ros::Subscriber accu_pc_sub_;
	ros::Publisher vehicle_pub_;
};


#endif /* STATIC_VEHICLE_RECOGNITION_H_ */
