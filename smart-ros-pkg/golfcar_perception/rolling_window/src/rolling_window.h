#ifndef ROLLING_WINDOW_H
#define ROLLING_WINDOW_H

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

using namespace std;

namespace golfcar_pcl{

	typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

	class rolling_window {
        public:
        rolling_window();
        ~rolling_window();        
    
        private:
        ros::NodeHandle nh_, private_nh_;
		string target_frame_, base_frame_;
        tf::TransformListener *tf_;

    	message_filters::Subscriber<sensor_msgs::LaserScan> 	laser_scan_sub_;
    	message_filters::Subscriber<sensor_msgs::PointCloud2> 	cloud_scan_sub_;
    	message_filters::Subscriber<nav_msgs::Odometry>			odom_sub_;
		tf::MessageFilter<sensor_msgs::LaserScan> 				*laser_scan_filter_;
	    tf::MessageFilter<sensor_msgs::PointCloud2> 			*cloud_scan_filter_;
	    tf::MessageFilter<nav_msgs::Odometry>					*odom_filter_;
		laser_geometry::LaserProjection projector_;
		
		//ros::Subscriber                             odom_sub_;
		tf::StampedTransform 	odom_OdomMeas_, scan_OdomMeas_, cloud_OdomMeas_;
		bool odom_init_, scan_init_, cloud_init_;
		float front_bound_, back_bound_;
		float odom_trigger_thresh_, scan_in_thresh_, cloud_in_thresh_;
		float angle_thresh_;
		int window_counts_;

        void scanCallback(const sensor_msgs::LaserScanConstPtr scan_in);
    	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr cloud_in);
		void odomCallback(const OdomConstPtr& odom);
		bool checkDistance(const tf::StampedTransform& oldTf, const tf::StampedTransform& newTf, 
															  float Dis_thresh);
		void windowProcessing(ros::Time current_time);

		PointCloud rolling_window_odom_, rolling_window_baselink_;

		ros::Publisher  rolling_window_pub_;
		
		//NF1
		ros::Publisher 	road_surface_pub_;

		//NF2
		geometry_msgs::Point32	viewpoint_td_sick_;
		void publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud);
		ros::Publisher	normals_poses_pub_;
    };

};

#endif

