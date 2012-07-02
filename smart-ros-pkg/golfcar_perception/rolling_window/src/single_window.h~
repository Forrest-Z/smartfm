#ifndef SINGLE_WINDOW_H
#define SINGLE_WINDOW_H

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

#include <laser_geometry/laser_geometry.h>

#include <fmutil/fm_math.h>

#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_clusters.h>

using namespace std;

namespace golfcar_pcl{

	typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

	class single_window {
        public:
        single_window();
        ~single_window();        
    
        private:
        ros::NodeHandle nh_, private_nh_;
        tf::TransformListener *tf_;

    	message_filters::Subscriber<sensor_msgs::PointCloud2> 	cloud_scan_sub_;
	    tf::MessageFilter<sensor_msgs::PointCloud2> 			*cloud_scan_filter_;
		
		PointCloud rolling_window_baselink_;
    	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr cloud_in);
		void publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud);
		void extractSurface(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, pcl::PointCloud<pcl::PointNormal>& norm_pcl);
		ros::Publisher	normals_poses_pub_, road_surface_pub_;
		geometry_msgs::Point32	viewpoint_td_sick_;
		double search_radius_, curvature_thresh_; 

		bool pub_surface_and_other_;
    };

};

#endif

