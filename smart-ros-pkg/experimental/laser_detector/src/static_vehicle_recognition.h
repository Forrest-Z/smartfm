/*
 * static_vehicle_recognition.h
 *
 *  Created on: Jun 5, 2013
 *      Author: Shen Xiaotong
 */

#ifndef STATIC_VEHICLE_RECOGNITION_H_
#define STATIC_VEHICLE_RECOGNITION_H_

#include <ros/ros.h>
#include "rolling_data_types.h"
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

namespace golfcar_perception{


typedef pcl::PointCloud<RollingPointXYZ> RollingPointCloud;
typedef pcl::PointCloud<RollingPointXYZNormal> RollingPointCloudNormal;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudNormal;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudRGBNormal;


class static_vehicle_recognition{
	ros::NodeHandle nh_;
	ros::Subscriber accu_pc_sub_;
	ros::Publisher vehicle_pub_;
	ros::Publisher normals_pub_;
	ros::Publisher plane_inlier_pub_;
	ros::Publisher plane_outlier_pub_;
	ros::Publisher ec_clusters_pub_;
	bool publish_normal_flag_;
public:
	static_vehicle_recognition(void);
	void accu_pc_callback(const RollingPointCloud::ConstPtr & accu_pc_in);
};

}

#endif /* STATIC_VEHICLE_RECOGNITION_H_ */
