
// void imageCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//   ROS_INFO("Received image captured at time %i.%i", msg->header.stamp.sec, msg->header.stamp.nsec);

//   cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
//   cv::Mat gray;
//   cv::cvtColor(frame, gray, CV_BGR2GRAY);

//   std::vector<cv::Point2f> features;
//   cv::goodFeaturesToTrack(gray, features, 1000, 0.01, 10);

//   /*
//   // into feature display
//   for (unsigned int i = 0; i< features.size(); ++i)
//   {
//     cv::circle(frame, features[i], 1, cv::Scalar(255, 255, 255), -1, CV_AA, 0 );
//   }
//   */

//   // tb.update(frame);

//   // publish tracks;

//   cv::imshow("camera", frame);
//   cv::waitKey(3);
// }
#include <ros/ros.h>
#include "FeatureViewerNode.hpp" // your code is in another file, Mario!

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_viewer");

  ROS_INFO("Starting feature viewer node.");

  FeatureViewerNode node;

  ros::spin();

  return(0);
}