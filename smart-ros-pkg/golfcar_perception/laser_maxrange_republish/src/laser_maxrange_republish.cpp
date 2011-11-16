/*
 * laser_maxrange_republish.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: golfcar
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher *laser_pub_;
void scanCallback(const sensor_msgs::LaserScan msg)
{
	sensor_msgs::LaserScan ls;
	ls = msg;
	double max_range_buffer = 0.5;
	for (unsigned int i=0;i<ls.ranges.size();i++)
	{
		if(ls.ranges[i]>ls.range_max)
		{
			ls.ranges[i]=ls.range_max-max_range_buffer;
		}
		else
		{
			ls.ranges[i]=ls.range_max+max_range_buffer;
		}
	}
	laser_pub_->publish(ls);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Publisher laser_pub= n.advertise<sensor_msgs::LaserScan>("scan_maxrange",1);
  laser_pub_ = &laser_pub;
  ros::Subscriber sub = n.subscribe("sick_scan", 1, scanCallback);
  ros::spin();

  return 0;
}
