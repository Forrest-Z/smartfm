#include "FeatureTrackerNode.hpp"

#include <infrastructure_camera_tracker/TrackedFeatureSet.h>

#include <cv_bridge/cv_bridge.h>

FeatureTrackerNode::FeatureTrackerNode(): node_handle("~"),
                                          image_transport(node_handle),
                                          state(sample)
{
  track_publisher = node_handle.advertise<infrastructure_camera_tracker::TrackedFeatureSet>("features", 1000);
  image_subscriber = image_transport.subscribe("image", 100, &FeatureTrackerNode::imageCallback, this);
  // state = sample;
};

FeatureTrackerNode::~FeatureTrackerNode()
{

};

void FeatureTrackerNode::imageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  cv::Mat frame = cv_bridge::toCvCopy(image, "bgr8")->image;
  header = image->header;
  double timestamp = image->header.stamp.toSec();
  ROS_INFO("Received image at time %f", timestamp);
  if ( sample == state)
  {
    ROS_INFO("(re)starting tracking.");
    tracker.restart(frame, timestamp);
    // publish new set of features
    publishFeatures();
    state = track;
  }
  else if ( track == state )
  {
    ROS_INFO("Tracking features.");
    // compute optical flow
    tracker.update(frame, timestamp);
    // publish
    publishFeatures();
  }
  else
  {
  }


  if(10 > tracker.getNumberOfActiveFeatures())
  {
    state = sample;
  }
};

void FeatureTrackerNode::publishFeatures(void)
{
  unsigned int n = tracker.getNumberOfFeatures();
  infrastructure_camera_tracker::TrackedFeatureSet message;
  std::vector<infrastructure_camera_tracker::TrackedFeature> tracked_features(n);

  std::vector<cv::Point2f> features = tracker.getFeatures();
  std::vector<uint64> ids = tracker.getIDs();
  std::vector<bool> activity = tracker.getActivity();
  double timestamp = tracker.getTime();

  for (unsigned int i = 0; i < n; ++i)
  {
    tracked_features[i].position.x = features.at(i).x;
    tracked_features[i].position.y = features.at(i).y;
    tracked_features[i].time = timestamp;
    tracked_features[i].id = ids[i];
    tracked_features[i].active = activity[i];
  }

  message.header = header;
  message.features = tracked_features;

  track_publisher.publish(message);
};