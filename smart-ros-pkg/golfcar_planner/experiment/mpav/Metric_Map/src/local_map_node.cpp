/*
 * local_map_node.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: liuwlz
 */

#include <Metric_Map/local_map.h>

int main(int argc, char**argv){
	ros::init(argc, argv, "local_map");
	MPAV::LocalMap lm;
	ros::spin();
	return 0;
}
