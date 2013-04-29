#include "FeatureDetectorNode.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <infrastructure_camera_tracker/Feature.h>
#include <infrastructure_camera_tracker/FeatureSet.h>

FeatureDetectorNode::FeatureDetectorNode(): node_handle("~"),
                                            image_transport(node_handle)
{
  feature_publisher = node_handle.advertise<infrastructure_camera_tracker::FeatureSet>("features", 1000);
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

  // convert to cv::Mat
  cv::Mat frame = cv_bridge::toCvCopy(image, "bgr8")->image;

  // get features
  std::vector<cv::Point2f> feature_vector = feature_detector.getFeatures(frame);

  // build message
  infrastructure_camera_tracker::Feature feature;
  infrastructure_camera_tracker::FeatureSet feature_set_msg;
  feature_set_msg.header = image->header;
  double t_now = image->header.stamp.toSec();
  feature.time = t_now;
  for (unsigned int i = 0; i < feature_vector.size(); ++i)
  {
    feature.position = cvPointToROSPoint(feature_vector[i]);
    feature_set_msg.features.push_back( feature );
  }

  // publish
  feature_publisher.publish(feature_set_msg);
};
