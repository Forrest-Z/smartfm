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
#include <fmutil/fm_stopwatch.h>
#include <pcl/filters/conditional_removal.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "geometry_common.h"
#include "rolling_window/plane_coef.h"
#include "rolling_window/pcl_indices.h"

namespace golfcar_pcl{

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

	class road_surface {
        public:
        road_surface();
        ~road_surface();   
	
	ros::NodeHandle nh_, private_nh_;
	tf::TransformListener *tf_;
	string odom_frame_, base_frame_;
	

	message_filters::Subscriber<PointCloud> 			*rolling_pcl_sub_;
    	message_filters::Subscriber<rolling_window::pcl_indices> 	*pcl_indices_sub_;
	message_filters::TimeSynchronizer<PointCloud, rolling_window::pcl_indices> *sync_;
	message_filters::Subscriber<nav_msgs::Odometry>			odom_sub_;
	tf::MessageFilter<nav_msgs::Odometry>			*odom_filter_;

	void pclCallback(const PointCloud::ConstPtr& pcl_in, const rolling_window::pcl_indices::ConstPtr &indices_in);
	void odomCallback(const OdomConstPtr& odom);
	
	PointCloud surface_pts_, boundary_pts_;
	bool input_update_flag_;
	PointCloud road_surface_odom_, road_boundary_odom_;
	vector <size_t> surface_index_batches_, boundary_index_batches_;
	size_t batchNum_limit_;
	

	vector<pcl::PointXYZ> poly_ROI_;
	
	ros::Publisher 	road_surface_pub_;
	ros::Publisher	road_boundary_pub_;
	ros::Publisher	fitting_plane_pub_;
	ros::Publisher	plane_coef_pub_;
	ros::Publisher  process_fraction_pub_;
	
	void publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud);
	ros::Publisher	normals_poses_pub_;

	void surface_extraction (const PointCloud &cloud_in, const rolling_window::pcl_indices & proc_indices,
	    			 PointCloud & surface_pts, PointCloud & boundary_pts);
    
	void planefitting_ROI(PointCloud & surface_pts, vector<pcl::PointXYZ> & poly_ROI,
			        	    PointCloud & fitting_plane, rolling_window::plane_coef & plane_coef);
	void pclXYZ_transform(string target_frame, PointCloud &pcl_src, PointCloud &pcl_dest);

	geometry_msgs::Point32	viewpoint_td_sick_;
	pcl::PointXYZ seedPoint_;
	double search_radius_, curvature_thresh_; 

	ros::Publisher 	surface_all_pub_;
	ros::Publisher	boundary_all_pub_;
    };

};

#endif
