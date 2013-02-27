#ifndef BRIGHTNESS_CONTROL_H
#define BRIGHTNESS_CONTROL_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <tf/transform_listener.h>
#include <cstdio>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <nav_msgs/Odometry.h>

#include "image_brightness_control/image_brightness.h"
#include "image_brightness_control/control_command.h"

using namespace std;
using namespace ros;
using namespace tf;

namespace golfcar_vision{

    class brightness_control {
        public:
        brightness_control();
        ~brightness_control();        
    
        private:
        ros::NodeHandle nh_, private_nh_;
        image_transport::ImageTransport it_;
        image_transport::CameraSubscriber cam_sub_;
	sensor_msgs::CvBridge bridge_;
	void ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg,
					   const sensor_msgs::CameraInfoConstPtr& info_msg);
   	ros::Publisher image_brightness_pub;
	ros::Publisher control_command_pub;
	int control_command_;
	int expected_centroid_;
	bool visualization_flag_;
	void DrawHistogram(IplImage* imgHist, CvHistogram *hist);
	void generate_ctrl_cmd(image_brightness_control::image_brightness & brightness_indicator);
	double last_err;
	double cmd;
	double k_p, k_i;
	int ctrl_gain;
    };
};

#endif
