#include "FeatureDetectorNode.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <infrastructure_camera_tracker/Features.h>

FeatureDetectorNode::FeatureDetectorNode(): node_handle("~"),
                                            image_transport(node_handle)
{
  feature_publisher = node_handle.advertise<infrastructure_camera_tracker::Features>("features", 1000);
  image_subscriber = image_transport.subscribe("image", 1, &FeatureDetectorNode::imageCallback, this);

  // to do: get parameters for the feature detector from the parameter server
};

FeatureDetectorNode::~FeatureDetectorNode()
{
  // do nothing
};

void FeatureDetectorNode::imageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  ROS_INFO("Received image captured at time %i.%i", image->header.stamp.sec, image->header.stamp.nsec);

  std::cout << "got image!\n";

  // convert to cv::Mat
  cv::Mat frame = cv_bridge::toCvCopy(image, "bgr8")->image;

  // get features
  std::vector<cv::Point2f> feature_vector = feature_detector.getFeatures(frame);

  // build message
  infrastructure_camera_tracker::Features features_msg;
  features_msg.header = image->header;
  for (unsigned int i = 0; i < feature_vector.size(); ++i)
  {
    features_msg.features.push_back( cvPointToROSPoint(feature_vector[i]) );
  }

  // publish
  feature_publisher.publish(features_msg);
};

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