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

//for line fitting in PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>



// for rand()
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <cmath>

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

class laser_scan_cluster{
public:
	pcl::PointCloud<pcl::PointXYZ> laser_cloud_;
	std::vector<int> index_;
	std::vector<double> distance_;
	std::vector<float> line_coef;
	int min_index_;
	int max_index_;
	int min_distance_;
	int max_distance_;
	void compute_minmax(void);
};

class single_scan_clustering{
public:
	single_scan_clustering(void);
	ros::NodeHandle nh_;
	ros::Subscriber laser_sub_;
	ros::Publisher clusters_pub_;

	ros::Publisher line_fitting_inlier_pub_;
	ros::Publisher line_fitting_outlier_pub_;

	ros::Publisher corners_pub_;

	sensor_msgs::PointCloud clusters_pc_;
	laser_geometry::LaserProjection laser_projection_;
	laser_scan_cloud laser_filtered_;



	void scan_callback(sensor_msgs::LaserScan::ConstPtr laser_in);
	void sac_line_fitting(sensor_msgs::LaserScan::ConstPtr laser_in);
	template <class PointT>
	inline double compute_dist(PointT a, PointT b);

};


}

#endif /* SINGLE_SCAN_CLUSTERING_H_ */
