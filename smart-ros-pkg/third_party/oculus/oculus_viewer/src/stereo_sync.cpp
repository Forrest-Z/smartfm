#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace message_filters;

class StereoSync
{
  image_transport::ImageTransport it_left_, it_right_, it_pub_;
  image_transport::SubscriberFilter image_left_sub_, image_right_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat sensorMsgsToCv(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    ROS_DEBUG("SensorMsgs to CV");
    cv_bridge::CvImagePtr cv_image;
    try {
	cv_image = cv_bridge::toCvCopy(msg_ptr, "bgr8");
    }
    catch (cv_bridge::Exception& e) {
	ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    return cv_image->image;
  }
      
  void syncCallback(const sensor_msgs::ImageConstPtr left_img_ptr, const sensor_msgs::ImageConstPtr right_img_ptr){
    assert(left_img_ptr->height == right_img_ptr->height);
    assert(left_img_ptr->width == right_img_ptr->width);
    int img_width = left_img_ptr->width;
    int img_height = left_img_ptr->height;
    cv::Mat left_img, right_img;
    left_img = sensorMsgsToCv(left_img_ptr);
    right_img = sensorMsgsToCv(right_img_ptr);
    
    cv::Mat combine_img (cv::Size(img_width*2, img_height), left_img.type());
    right_img.copyTo(combine_img(cv::Rect(0, 0, img_width, img_height)));
    left_img.copyTo(combine_img(cv::Rect(img_width, 0, img_width, img_height)));
//     cv::imshow("combined_img", combine_img);
//     cvWaitKey(1);
    
    cv_bridge::CvImage cv_img(left_img_ptr->header, left_img_ptr->encoding, combine_img);
    image_pub_.publish(cv_img.toImageMsg());
  }
public:
  StereoSync(ros::NodeHandle &n): it_left_(n), it_right_(n), it_pub_(n){
    image_left_sub_.subscribe(it_left_, "minoru_left/image_raw", 20);
    image_right_sub_.subscribe(it_right_, "minoru_right/image_raw", 20);
    image_pub_ = it_pub_.advertise("minoru_sync/image_raw", 20);
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> imageSyncPolicy;
    Synchronizer<imageSyncPolicy> imageSync(imageSyncPolicy(20), image_left_sub_, image_right_sub_);
    imageSync.registerCallback(boost::bind(&StereoSync::syncCallback, this, _1, _2));
    ros::spin();
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "stereo_sync");
  ros::NodeHandle n;
  StereoSync ss(n);
  return 0;
}