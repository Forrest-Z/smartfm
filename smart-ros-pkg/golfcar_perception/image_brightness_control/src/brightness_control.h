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
#include <dynamic_reconfigure/server.h>
#include "image_brightness_control/brightCTRConfig.h"

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
		dynamic_reconfigure::Server<image_brightness_control::brightCTRConfig>  *srv;

		void ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
					   
		ros::Publisher image_brightness_pub;
		ros::Publisher control_command_pub;

		bool visualization_flag_;
		void DrawHistogram(IplImage* imgHist, CvHistogram *hist);
		double last_err, last_err2;
		double cmd;
		double k_p, k_i, k_d;

		double p_shutter_, i_shutter_, d_shutter_;
		double p_gain_, i_gain_, d_gain_;
		int shutter_value_, gain_value_, expected_centroid_;
		bool shutter_control_, gain_control_, visualization_;
		void reconfig(image_brightness_control::brightCTRConfig &config, uint32_t level);
		image_brightness_control::control_command control_command_;
		void generate_ctrl_cmd(image_brightness_control::image_brightness & brightness_indicator);
		void shutter_control(int brightness_centroid);
		void gain_control(int brightness_centroid);
		};
};

#endif
