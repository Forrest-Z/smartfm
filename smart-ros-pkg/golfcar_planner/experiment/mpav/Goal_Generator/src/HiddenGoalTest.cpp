/*
 * HiddenGoalTest.cpp
 *
 *  Created on: Nov 23, 2013
 *      Author: liuwlz
 */

#include <ros/ros.h>
#include <Goal_Generator/HiddenGoal.hpp>

using namespace MPAV;

int main(int argc, char** argv){
	ros::init(argc, argv, "HiddenGoal");
	HiddenGoal hidden;
	ros::spin();
	return 1;
}
