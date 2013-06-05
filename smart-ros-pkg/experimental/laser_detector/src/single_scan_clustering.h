/*
 * single_scan_clustering.h
 *
 *  Created on: Jun 5, 2013
 *      Author: sxt
 */

#ifndef SINGLE_SCAN_CLUSTERING_H_
#define SINGLE_SCAN_CLUSTERING_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <laser_geometry/laser_geometry.h>



namespace golfcar_perception{

using namespace std;

struct laser_scan {
	vector<double> angles;
	vector<double> dists;
};

class laser_scan_cloud{
public:
	laser_scan scan_;
	sensor_msgs::PointCloud pc_;
	void update(const sensor_msgs::LaserScan raw_scan);
};

class single_scan_clustering{
public:
	single_scan_clustering(void);
	ros::NodeHandle nh_;
	ros::Subscriber laser_sub_;
	ros::Publisher clusters_pub_;

	sensor_msgs::PointCloud clusters_pc_;
	laser_geometry::LaserProjection laser_projection_;
	laser_scan_cloud laser_filtered_;
	void scan_callback(sensor_msgs::LaserScan::ConstPtr laser_in);

};


}

#endif /* SINGLE_SCAN_CLUSTERING_H_ */
