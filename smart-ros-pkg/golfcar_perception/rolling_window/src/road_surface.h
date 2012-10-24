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
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>

#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>

#include <laser_geometry/laser_geometry.h>

#include <fmutil/fm_math.h>

#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <math.h>

//for plane fitting fuction still use "PointCloud" data type;
//something wrong with "sac_model_stick.hpp" when compiling;

/*
#include <pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_registration.hpp>
#include <pcl/sample_consensus/impl/sac_model_circle.hpp>
#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>
#include <pcl/sample_consensus/impl/sac_model_stick.hpp>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
*/

#include <pcl/search/impl/organized.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <fmutil/fm_stopwatch.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "geometry_common.h"
#include "rolling_window/plane_coef.h"
#include "rolling_window/pcl_indices.h"
#include <cstdlib>
#include "rolling_data_types.h"
#include <pcl_ros/impl/transforms.hpp>

namespace golfcar_pcl{

	typedef struct
	{
		size_t BDptNum;
		float  longest_horizontal_dist;
		float  pitch_speed;
		float  roll_speed;
	}boundary_scan_info;

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
	typedef pcl::PointCloud<pcl::PointNormal> PointCloudNormal;
	typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudRGBNormal;

	typedef pcl::PointCloud<RollingPointXYZ> RollingPointCloud;
	typedef pcl::PointCloud<RollingPointXYZNormal> RollingPointNormalCloud;

	typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

	class road_surface {
        public:
        road_surface();
        ~road_surface();   
	
	ros::NodeHandle nh_, private_nh_;
	tf::TransformListener *tf_;
	string odom_frame_, base_frame_;
	

	message_filters::Subscriber<RollingPointCloud> 			*rolling_pcl_sub_;
    message_filters::Subscriber<rolling_window::pcl_indices> 	*pcl_indices_sub_;
	message_filters::TimeSynchronizer<RollingPointCloud, rolling_window::pcl_indices> *sync_;
	message_filters::Subscriber<nav_msgs::Odometry>			odom_sub_;
	tf::MessageFilter<nav_msgs::Odometry>			*odom_filter_;

	void pclCallback(const RollingPointCloud::ConstPtr& pcl_in, const rolling_window::pcl_indices::ConstPtr &indices_in);
	void odomCallback(const OdomConstPtr& odom);
	
	RollingPointCloud surface_pts_, boundary_pts_;
	bool input_update_flag_;

	//maintained for plane fitting purposes;
	RollingPointCloud road_surface_odom_, road_boundary_odom_;

	//important data management;
	vector <RollingPointCloud> raw_pcl_batches_;
	vector <RollingPointNormalCloud> pcl_normal_batches_;
	vector <vector <int> > surface_index_batches_, boundary_index_batches_, outlier_index_batches_;

	size_t batchNum_limit_;
	
	vector<pcl::PointXYZ> poly_ROI_;
	
	ros::Publisher 	road_surface_pub_;
	ros::Publisher	road_boundary_pub_;
	ros::Publisher	fitting_plane_pub_;
	ros::Publisher	plane_coef_pub_;
	ros::Publisher  process_fraction_pub_;
	
	void publishNormal(RollingPointNormalCloud& pcl_cloud);
	ros::Publisher	normals_poses_pub_;

	void surface_extraction (const RollingPointCloud &cloud_in, const rolling_window::pcl_indices & proc_indices,
							RollingPointCloud & surface_pts, RollingPointCloud & boundary_pts);
    
	void planefitting_ROI(RollingPointCloud & surface_pts, vector<pcl::PointXYZ> & poly_ROI,
							PointCloud & fitting_plane, rolling_window::plane_coef & plane_coef);

	pcl::PointXYZ	viewpoint_td_sick_;
	RollingPointXYZ seedPoint_;

	double search_radius_, curvature_thresh_; 

	ros::Publisher 	surface_all_pub_;
	ros::Publisher	boundary_all_pub_;

	bool planefitting_init_, clustering_init_;
	tf::StampedTransform 	planefitting_OdomMeas_, clustering_OdomMeas_;
	double planefitting_disThresh_,clustering_disThresh_;
	bool checkDistance(const tf::StampedTransform& oldTf, const tf::StampedTransform& newTf, float Dis_thresh);

	ros::Publisher  clusters_pub_, normal_visual_pub_, surface_slope_pub_;
	//ros::Publisher planes_pub_, pcl_cloud_restPub_;

	vector<float> jet_r_, jet_g_, jet_b_;
	double curvature_visual_limit_;
	double normalZ_visual_limit_;
	bool curvature_visualization_, normalZ_visualization_;

	inline void colormap_jet(RollingPointXYZNormal& point_in, pcl::PointXYZRGBNormal &point_out);
	void road_slope_visualization(RollingPointCloud & surface_pts, RollingPointCloud & boundary_pts);
    };

};

#endif
