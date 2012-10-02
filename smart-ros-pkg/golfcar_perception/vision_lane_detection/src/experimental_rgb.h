#ifndef EXPERIMENTAL_RGB_H
#define EXPERIMENTAL_RGB_H

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
#include "image_proc.h"
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

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

namespace golfcar_vision{

    class experimental_rgb {
        public:
        experimental_rgb();
        ~experimental_rgb();        
    
        private:
        ros::NodeHandle nh_, private_nh_;
        image_transport::ImageTransport it_;
        image_transport::CameraSubscriber cam_sub_;
        sensor_msgs::CvBridge bridge_;
        tf::TransformListener tf_;
        image_geometry::PinholeCameraModel cam_model_;
        string odom_frame_, base_frame_;

	message_filters::Subscriber<PointCloud> 	cloud_scan_sub_;
    	tf::MessageFilter<PointCloud> 			*cloud_scan_filter_;       

	bool fixedTf_inited_;
	bool process_pcl_;
        tf::Transform src_dest_tf_;   
	PointCloud pcl_batch_;
	ros::Publisher  rbg_pub_;

        void ImageCallBack(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
	void curbPts_to_image(PointCloud &pts_3d,  PointCloudRGB &rgb_pts, IplImage* color_image);
	void pclCallback(const PointCloud::ConstPtr& pcl_in);
	void pclXYZ_transform(string target_frame, const PointCloud &pcl_src, PointCloud &pcl_dest);
    };
};

#endif
