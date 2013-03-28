#ifndef GOLFCAR_VISION_CP_PROJECTOR_H
#define GOLFCAR_VISION_CP_PROJECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cstdio>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include "lane_marker_common.h"
#include "rolling_window/plane_coef.h"
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "vehicle_model.hpp"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

namespace golfcar_vision{

    class CP_projector {
        public:
    	CP_projector();
        ~CP_projector();

	private:
	ros::NodeHandle nh_, private_nh_;
	image_transport::ImageTransport it_;
	image_transport::CameraSubscriber cam_sub_;
	sensor_msgs::CvBridge bridge_;
	tf::TransformListener tf_;
	image_geometry::PinholeCameraModel cam_model_;

	string vehicle_ID_;
	string odom_frame_;
	string vehicle0_dest_frame_, vehicle0_Hdest_frame_;
	string vehicle1_dest_frame_, vehicle1_Hdest_frame_;

	message_filters::Subscriber<PointCloudRGB> 	cloud_scan_sub_;
    tf::MessageFilter<PointCloudRGB> 			*cloud_scan_filter_;

	message_filters::Subscriber<PointCloudRGB> 	cloud_scan_sub1_;
    tf::MessageFilter<PointCloudRGB> 			*cloud_scan_filter1_;

    message_filters::Subscriber<PointCloudRGB> 	cloud_scan_sub2_;
    tf::MessageFilter<PointCloudRGB> 			*cloud_scan_filter2_;

    PointCloudRGB pcl_batch_;

    void ImageCallBack(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
	void pclCallback(const PointCloudRGB::ConstPtr& pcl_in);
	void pcl_process(const PointCloudRGB::ConstPtr& pcl_in, string dest_camera_frame, IplImage *project_image, CvScalar color_plot);
	void merge_images(IplImage *dst_img, IplImage *src_img);

	int image_width_, image_height_;
	bool camera_model_initialized_;

	IplImage* project_image00, *project_image01, *project_image02;
	IplImage* project_image11, *project_image12;
	IplImage* project_image22;

	IplImage* project_imageH00, *project_imageH01, *project_imageH02;
	IplImage* project_imageH11, *project_imageH12;
	IplImage* project_imageH22;

	CvScalar vehicle1st_color, vehicle2nd_color;

    string own_image_name_, project_image_name_, merged_name_;
	double show_scale_;

	double vehicle_length_, vehicle_width_, vehicle_height_;
	vehicle_model *vehicle_box;

	double vehicle01_distance, vehicle02_distance, vehicle1st_velocity;
	double vehicle12_distance, vehicle2nd_velocity;

	ros::Subscriber vehicle1st_velo_sub_, vehicle2nd_velo_sub_;

	double distance_between_vehicles(std_msgs::Header vehicle1, std_msgs::Header vehicle2);

	void velocity1st_callback(const geometry_msgs::TwistStamped::ConstPtr& velo_in);
	void velocity2nd_callback(const geometry_msgs::TwistStamped::ConstPtr& velo_in);

    };
};

#endif
