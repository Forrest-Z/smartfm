#ifndef ROLLING_WINDOW_PCL_H
#define ROLLING_WINDOW_PCL_H

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
#include <pcl/filters/impl/voxel_grid.hpp>

#include <laser_geometry/laser_geometry.h>

#include <fmutil/fm_math.h>

#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "geometry_common.h"
#include "rolling_window/plane_coef.h"
#include "rolling_window/pcl_indices.h"

#include "rolling_data_types.h"
#include <sensor_msgs/Imu.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>


using namespace std;

namespace golfcar_pcl{

	typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<RollingPointXYZ> RollingPointCloud;

	class imu_info
	{
		public:
		float pitch_para1, pitch_para2;
		float roll_para;

		float pitch_speed, pitch;
		float roll;
	};

	class rolling_window_pcl {
        public:
        rolling_window_pcl();
        ~rolling_window_pcl();        
    
        private:
        ros::NodeHandle nh_, private_nh_;
        string odom_frame_, base_frame_;
        tf::TransformListener *tf_;

        golfcar_pcl::imu_info ms_imu_;

    	message_filters::Subscriber<sensor_msgs::LaserScan> 	laser_scan_sub_;
    	message_filters::Subscriber<nav_msgs::Odometry>		odom_sub_;
    	tf::MessageFilter<sensor_msgs::LaserScan> 		*laser_scan_filter_;
    	tf::MessageFilter<nav_msgs::Odometry>			*odom_filter_;

		ros::Subscriber                             imu_sub_;

		tf::StampedTransform 	odom_OdomMeas_, scan_OdomMeas_, cloud_OdomMeas_;
		bool odom_init_, scan_init_, cloud_init_;
		double front_bound_, back_bound_;
		double odom_trigger_thresh_, scan_in_thresh_, cloud_in_thresh_;
		float angle_thresh_;
		int window_counts_;

        void scanCallback(const sensor_msgs::LaserScanConstPtr scan_in);
		void odomCallback(const OdomConstPtr& odom);
		void imuCallback(const sensor_msgs::Imu::ConstPtr msg);
		bool checkDistance(const tf::StampedTransform& oldTf, const tf::StampedTransform& newTf, float Dis_thresh);
		void windowProcessing(ros::Time current_time);
	
		bool process_fraction_exist_;
		RollingPointCloud 		rolling_window_odom_, 	rolling_window_baselink_;
		RollingPointCloud      process_fraction_odom_, front_buffer_odom_;
		ros::Publisher  rolling_window_pub_, 	process_fraction_pub_;
		void pcl_downsample(RollingPointCloud &point_cloud);

		size_t laser_scan_serial_;
    };

};

#endif

