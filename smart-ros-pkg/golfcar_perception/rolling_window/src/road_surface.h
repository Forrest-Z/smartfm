#ifndef ROAD_SURFACE_PCL_H
#define ROAD_SURFACE_PCL_H

/*
 *  Created on: 27/08/2012
 *  Author: Baoxing
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <message_filters/subscriber.h>

#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <laser_geometry/laser_geometry.h>

#include <fmutil/fm_math.h>

#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "geometry_common.h"
#include "rolling_window/plane_coef.h"
#include <fmutil/fm_stopwatch.h>
#include <pcl/filters/conditional_removal.h>

namespace golfcar_pcl{

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

	class road_surface {
        public:
        road_surface();
        ~road_surface();   
	
	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber	rolling_pcl_sub_;
	void pclCallback(const PointCloud::ConstPtr& pcl_in);
	vector<pcl::PointXYZ> poly_ROI_;

	ros::Publisher 	road_surface_pub_;
	ros::Publisher	road_boundary_pub_;
	ros::Publisher	fitting_plane_pub_;
	ros::Publisher	plane_coef_pub_;
	
	
	void publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud);
	ros::Publisher	normals_poses_pub_;

	void surface_extraction (const PointCloud &cloud_in, bool window_process, vector<pcl::PointXYZ> & poly_ROI,
				 pcl::PointCloud<pcl::PointNormal> & point_normals,
	    			 PointCloud & surface_pts, PointCloud & boundary_pts, 
				 PointCloud & fitting_plane, rolling_window::plane_coef & plane_coef);
    
	void planefitting_ROI(PointCloud & surface_pts, vector<pcl::PointXYZ> & poly_ROI,
			        	    PointCloud & fitting_plane, rolling_window::plane_coef & plane_coef);
	
	geometry_msgs::Point32	viewpoint_td_sick_;
	pcl::PointXYZ seedPoint_;
	double search_radius_, curvature_thresh_; 
    };

};

#endif
