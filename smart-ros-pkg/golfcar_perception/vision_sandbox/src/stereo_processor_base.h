#ifndef GOLFCART_VISION_STEREO_FEATURES_H_
#define GOLFCART_VISION_STEREO_FEATURES_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <stereo_msgs/DisparityImage.h>
#include <image_geometry/stereo_camera_model.h>

//learn from package (1) stereo_image_proc: pointcloud.cpp;
//					 (2) viso2_ros: stereo_processor.h & stereo_odometer.cpp;

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

namespace golfcart_vision
{

class StereoProcessor
{

protected:
	boost::shared_ptr<image_transport::ImageTransport> it_;

	// Subscriptions
	image_transport::SubscriberFilter sub_l_image_;
	message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
	message_filters::Subscriber<DisparityImage> sub_disparity_;
	typedef ExactTime<Image, CameraInfo, CameraInfo, DisparityImage> ExactPolicy;
	typedef ApproximateTime<Image, CameraInfo, CameraInfo, DisparityImage> ApproximatePolicy;
	typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
	boost::shared_ptr<ExactSync> exact_sync_;
	boost::shared_ptr<ApproximateSync> approximate_sync_;

	boost::mutex connect_mutex_;
	image_geometry::StereoCameraModel model_;

public:

  StereoProcessor()
  {
	  ros::NodeHandle private_nh("~");
	  ros::NodeHandle nh;

	std::string left_topic, left_info_topic, right_info_topic, disparity_topic;
    private_nh.param("left_topic", left_topic, std::string("/stereo_camera/left/image_rect_color"));
    private_nh.param("left_info_topic", left_info_topic, std::string("/stereo_camera/left/camera_info"));
    private_nh.param("right_info_topic", right_info_topic, std::string("/stereo_camera/right/camera_info"));
    private_nh.param("disparity_topic", disparity_topic, std::string("/stereo_camera/disparity"));

	int queue_size;
	private_nh.param("queue_size", queue_size, 5);
	bool approx;
	private_nh.param("approximate_sync", approx, false);

	it_.reset(new image_transport::ImageTransport(nh));
    sub_l_image_  .subscribe(*it_, left_topic, 1);
    sub_l_info_   .subscribe(nh,   left_info_topic, 1);
    sub_r_info_   .subscribe(nh,   right_info_topic, 1);
    sub_disparity_.subscribe(nh,   disparity_topic, 1);

    if (approx)
    {
    	approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size),sub_l_image_, sub_l_info_,sub_r_info_, sub_disparity_));
    	approximate_sync_->registerCallback(boost::bind(&StereoProcessor::dataCb, this, _1, _2, _3, _4));
    }
    else
	{
    	exact_sync_.reset( new ExactSync(ExactPolicy(queue_size), sub_l_image_, sub_l_info_, sub_r_info_, sub_disparity_));
    	exact_sync_->registerCallback(boost::bind(&StereoProcessor::dataCb, this, _1, _2, _3, _4));
	}

  }

  /**
   * Implement this method in sub-classes
   */

  void dataCb(const ImageConstPtr& l_image_msg,
          const CameraInfoConstPtr& l_info_msg,
          const CameraInfoConstPtr& r_info_msg,
          const DisparityImageConstPtr& disp_msg)
  {
    // call implementation
	  imageCb(l_image_msg, l_info_msg, r_info_msg, disp_msg);
  }

  virtual void imageCb(const ImageConstPtr& l_image_msg,
          const CameraInfoConstPtr& l_info_msg,
          const CameraInfoConstPtr& r_info_msg,
          const DisparityImageConstPtr& disp_msg) = 0;
};

} // end of namespace

#endif

