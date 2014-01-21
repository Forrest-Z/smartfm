/*
 * main.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: liuwlz
 */

#include <Automoton_Control/ObstAvoid/ObstAvoidNode.hpp>

using namespace MPAV;

int main(int argc, char**argv){
	ros::init(argc, argv, "ObstAvoid");
	ObstAvoid inter;
	ros::spin();
	return 1;
}
