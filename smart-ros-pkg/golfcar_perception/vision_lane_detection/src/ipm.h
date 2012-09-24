#ifndef LANE_MARKER_IPM_H
#define LANE_MARKER_IPM_H

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

using namespace std;
using namespace ros;
using namespace tf;

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

namespace golfcar_vision{

    class ipm {
        public:
        ipm();
        ~ipm();        
    
        private:
        ros::NodeHandle nh_, private_nh_;
        
        IplImage *ipm_image_;
		  double camera_baselink_dis_;
		  double ipm_center_x_, ipm_center_y_;
		  double ipm_ROI_height_, ipm_ROI_near_width_, ipm_ROI_far_width_;
		  double scale_;
		  
        image_transport::ImageTransport it_;
        image_transport::CameraSubscriber cam_sub_;
        image_transport::Publisher image_pub_;
        sensor_msgs::CvBridge bridge_;
        tf::TransformListener tf_;

        image_geometry::PinholeCameraModel cam_model_;
        
        std::string dest_frame_id_;
        bool fixedTf_inited_;
        //here "camera" is the source frame as global frame in usual cases; 
        //"dest" is the target frame as usual base frame;
        tf::Transform src_dest_tf_;   
        
        //"gndPt_" is the real position in "baselink" frame;
        //"dstQuad_" is the image representation of "gndPt_"; and at the same time, the dest image of "wrap" operation; 
        //they are two different representations about the ground; 
        //"srcQuad_" is pixels in camera image;
        CvPoint2D32f gndQuad_[4], srcQuad_[4], dstQuad_[4];
		  CvMat *warp_matrix_, *projection_matrix_; 

        //scale denotes the ratio of pixel/distance in "image_ipm";
        //float scale_;
        sensor_msgs::CameraInfo CameraStaticInfo_;
        int training_frame_serial_;

        void ImageCallBack(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
        void GndPt_to_Src(CvPoint2D32f * gnd_pointer, CvPoint2D32f* src_pointer);
        void GndPt_to_Dst(CvPoint2D32f * gnd_pointer, CvPoint2D32f* dst_pointer);
        void Wrap_Src_to_Dst();

        image_proc* image_processor_;
        
        vision_lane_detection::markers_info markers_, markers_2nd_;
        ros::Publisher markers_info_pub;
	     ros::Publisher markers_info_2nd_pub;

		  //ros::Subscriber  odom_sub_;
	     tf::Transformer  transformer_;
	     tf::StampedTransform odom_meas_, odom_meas_old_;
	     bool pub_init_;
    	  bool publish_flag_;
    	  //void odomCallback(const OdomConstPtr& odom);
	     void process_control(ros::Time meas_time);
	     double  publish_dis_thresh_;
    	  double  publish_angle_thresh_;

		  bool visualization_flag_;

		  ros::Subscriber                             planeCoef_sub_;
	     void planeCoefCallback(const rolling_window::plane_coef::ConstPtr& coef_in);
	     rolling_window::plane_coef 		    plane_ROI_;

		  //to accumulate the curb points (road_boundary);
	     string odom_frame_, base_frame_;
	     message_filters::Subscriber<sensor_msgs::PointCloud> 	curb_point_sub_;
	     tf::MessageFilter<sensor_msgs::PointCloud> 		*curb_point_filter_;
	     void curbCallback(const sensor_msgs::PointCloudConstPtr  curb_in);
	     void pcl_to_RawImage(sensor_msgs::PointCloud &pts_3d, std::vector <CvPoint2D32f> & pts_image);
	     void IpmImage_to_pcl(std::vector <CvPoint2D32f> & pts_image, sensor_msgs::PointCloud &pts_3d);
	
	     sensor_msgs::PointCloud left_accumulated_, right_accumulated_;
	     size_t curb_num_limit_;
	     ros::Publisher  left_pub_, right_pub_;
	
		  vision_lane_detection::conti_lanes lanes_inImg_;
		  ros::Publisher  lanes_pub_;
		  ros::Publisher  lanes_ptcloud_pub_;
		  
		  bool odom_control_;
   };
};

#endif
