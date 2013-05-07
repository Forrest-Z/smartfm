#include "FeatureTrackerNode.hpp"

#include <infrastructure_camera_tracker/TrackedFeatureSet.h>

#include <cv_bridge/cv_bridge.h>

FeatureTrackerNode::FeatureTrackerNode(): node_handle("~"),
                                          image_transport(node_handle),
                                          state(sample)
{
  track_publisher = node_handle.advertise<infrastructure_camera_tracker::TrackedFeatureSet>("features", 1000);
  image_subscriber = image_transport.subscribe("image", 100, &FeatureTrackerNode::imageCallback, this);
};

FeatureTrackerNode::~FeatureTrackerNode()
{

};

void FeatureTrackerNode::imageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  cv::Mat frame = cv_bridge::toCvCopy(image, "bgr8")->image;
  header = image->header;
  double timestamp = image->header.stamp.toSec();
  ROS_DEBUG("Received image at time %f", timestamp);
  if ( sample == state )
  {
    tracker.restart(frame, timestamp);
    time_last_restart = timestamp;
    ROS_INFO("(re)starting tracking, %i features found", tracker.getNumberOfFeatures());
    state = track;
    publishFeatures(true);                                                      // publish new set of features
  }
  else if ( track == state )
  {
    ROS_DEBUG("Tracking features.");
    tracker.update(frame, timestamp);                                           // compute optical flow
    publishFeatures(false);                                                     // publish update
  }
  else
  {
    ROS_ERROR("Unexpected state!");
  }

  // resampling criteria
  if(10 > tracker.getNumberOfActiveFeatures() || 3 < (timestamp - time_last_restart))
  {
    state = sample;
  }
};

void FeatureTrackerNode::publishFeatures(bool new_set)
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
  message.new_set = new_set;

  track_publisher.publish(message);
};