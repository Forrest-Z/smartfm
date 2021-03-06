#ifndef VISION_CP_H
#define VISION_CP_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <cstdio>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include "vision_cp/chess_board_poses.h"
#include <fmutil/fm_stopwatch.h>

using namespace std;
using namespace ros;
using namespace tf;

namespace golfcar_vision{

    //different chess_board model;
    class chess_board {
        public:      
	int board_w;
	int board_h;
	double side_length;
	std::string board_name; 
    };

    class visionCP {
        public:
        visionCP();
        ~visionCP();        
    
        private:
        ros::NodeHandle nh_, private_nh_;
        image_transport::ImageTransport it_;
        image_transport::CameraSubscriber cam_sub_;
        sensor_msgs::CvBridge bridge_;
        image_geometry::PinholeCameraModel cam_model_;
        sensor_msgs::CameraInfo CameraStaticInfo_;
        
        double rect_height_, rect_width_;
        void ImageCallBack(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
		  bool camera_init_;
		  CvMat* intrinsic_matrix_;
		  CvMat* distortion_coeffs_;
	     ros::Publisher board_pub_;
		  void sort_pts(std::vector <CvPoint> & vertices);
		  void calc_cb_pose( const sensor_msgs::CameraInfoConstPtr& info_msg, std::string board_name,
										CvMat* obj_pts, CvMat* img_pts, geometry_msgs::Pose & board_pose );
		  bool Extract_Rects (IplImage* src, const sensor_msgs::CameraInfoConstPtr& info_msg, 
									 std::string board_name, geometry_msgs::Pose & board_pose);
   	  bool check_angles(std::vector <CvPoint> vetices);
		  //this can only work as a pointer, or some tricky things will happen;
		  tf::TransformBroadcaster *tfb_;
		  ros::Duration transform_tolerance_;
		  
		  int binary_thresh_;
		  int cvpoly_thresh_;
		  double angle_thresh_;
		  double ratio_thresh_;
		  bool red_detect_, blue_detect_;
		  bool visual_flag_;
    };


};

#endif
