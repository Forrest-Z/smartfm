/*
 * correlative_matching.cpp
 *
 *  Created on: May 8, 2012
 *      Author: demian
 */

#include "correlative_matching.hpp"
#include <ros/ros.h>
#include "pcl/ros/conversions.h"

class cm_node
{
public:
	cm_node();
	ros::Subscriber pointcloud_sub_;
	ros::Publisher low_res_map_pub_;
	ros::NodeHandle nh_;
	correlative_matching cm_;
	void cloudCallback(sensor_msgs::PointCloud2 cloud_in);
};

cm_node::cm_node() : cm_(40.0, 0.3, 0.03)
{
	pointcloud_sub_ = nh_.subscribe("sickldmrs/cloud", 10, &cm_node::cloudCallback, this);
	low_res_map_pub_ = nh_.advertise<sensor_msgs::PointCloud>("low_res_map", 10);
	ros::spin();

}

void cm_node::cloudCallback(sensor_msgs::PointCloud2 cloud_in)
{
	pcl::PointCloud<pcl::PointXYZ> pcl_in;
	pcl::fromROSMsg(cloud_in, pcl_in);
	cm_.getBestMatch(pcl_in);
	sensor_msgs::PointCloud low_res_map;
	cm_.getLowResMap(low_res_map);
	low_res_map.header = cloud_in.header;
	low_res_map_pub_.publish(low_res_map);
}

int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "cm_node");
	cm_node cmNode;
	return 0;
}
