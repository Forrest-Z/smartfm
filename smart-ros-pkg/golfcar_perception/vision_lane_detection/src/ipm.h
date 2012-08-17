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


#include "lane_marker_common.h"
#include "image_proc.h"

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

        //scale denotes the ratio of pixel/distance in "image_ipm";
        float scale_;
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
    };
};

#endif
