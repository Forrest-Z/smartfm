#ifndef GOLFCAR_VISION_CP_PROJECTOR_H
#define GOLFCAR_VISION_CP_PROJECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <cstdio>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
        string odom_frame_, dest_frame_id_;



	message_filters::Subscriber<PointCloudRGB> 	cloud_scan_sub_;
    tf::MessageFilter<PointCloudRGB> 			*cloud_scan_filter_;
    PointCloudRGB pcl_batch_;

    void ImageCallBack(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
	void pclCallback(const PointCloudRGB::ConstPtr& pcl_in);

	int image_width_, image_height_;
	bool camera_model_initialized_;
	IplImage* color_image_;

    string own_image_name_, project_image_name_, merged_name_;
	double show_scale_;

	double vehicle_length_, vehicle_width_, vehicle_height_;
	vehicle_model *vehicle_box;
	bool visualize_farest_predecessor_;
	string predecessor_frameID_;
	int predecessor_color_mode_;
    };
};

#endif
