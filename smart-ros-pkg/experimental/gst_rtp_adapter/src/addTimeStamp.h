#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

sensor_msgs::Image addTimeStamp(sensor_msgs::Image& msg, int y_offset)
{
  ros::Time time_now = ros::Time::now();
  ROS_DEBUG("SensorMsgs to CV");
  cv_bridge::CvImagePtr cv_image;
  try {
      cv_image = cv_bridge::toCvCopy(msg, "rgb8");
  }
  catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  int epochSec = time_now.toSec();
  
  time_t raw_time = epochSec;
  struct tm* timeinfo;
  timeinfo = localtime(&raw_time);
  string time_str = asctime(timeinfo);
  stringstream ss;
  ss<<time_str<<" "<<(time_now.toSec() - epochSec)*1000;
  
  cv::putText(cv_image->image, ss.str(), cv::Point(0, msg.height-y_offset), 
	      cv::FONT_HERSHEY_PLAIN, 0.8,cvScalar(0,0,0), 1, 8);
  
  cv_bridge::CvImage cv_img(msg.header, msg.encoding, cv_image->image);
  return *cv_img.toImageMsg();
};