#include "color_tracking.h"

color_tracking::color_tracking(ros::NodeHandle &n) : private_nh_("~"), n_(n), it_(n)
{
  image_sub_ = it_.subscribe("/camera_front/image_raw", 1, &color_tracking::imageCallback, this);
  image_pub_ = it_.advertise("/camera_front/tracking_result", 1);
  
  cv::namedWindow("CameraOutput",WINDOW_AUTOSIZE);
  cv::namedWindow("Object",WINDOW_AUTOSIZE);
  
}

color_tracking::~color_tracking()
{
  destroyAllWindows();
}

void color_tracking::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image, "bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  cv_image_->image.copyTo(frame1_rgb_);
  
  cv::Mat flow_rgb;
  frame1_rgb_.copyTo(flow_rgb);
  cv::imshow("CameraOutput",flow_rgb);
  
  cv_image_ -> image = flow_rgb;
  image_pub_.publish(cv_image_->toImageMsg());
  
  cv:: waitKey(3);
}

int main(int argc, char** argv)
{
  std::cout << "Hello there!" << std::endl;
  ros::init(argc, argv, "color_tracking");
  ros::NodeHandle n;
  color_tracking color_tracking_node(n);
  ros::spin();
  return 0;
}