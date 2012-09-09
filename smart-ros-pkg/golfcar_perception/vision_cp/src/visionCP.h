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
	
	chess_board board_1st_;
	chess_board board_2nd_;
        void ImageCallBack(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
	bool camera_init_;
	CvMat* intrinsic_matrix_;
	CvMat* distortion_coeffs_;
	void calc_cb_pose(CvMat* obj_pts, CvMat* img_pts, geometry_msgs::Pose & board_pose );
	ros::Publisher board_pub_;
    };


};

#endif
