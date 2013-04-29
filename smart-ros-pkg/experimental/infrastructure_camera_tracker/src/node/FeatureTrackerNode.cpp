#include "FeatureTrackerNode.hpp"

FeatureTrackerNode::FeatureTrackerNode(): node_handle("~"),
                                          image_transport(node_handle),
                                          feature_subscriber(node_handle, "features", 1)
{
  track_publisher = node_handle.advertise<infrastructure_camera_tracker::TrackSet>("tracks", 1000);

  image_subscriber = image_transport.subscribe("image", 1, &FeatureTrackerNode::imageCallback, this);

};

FeatureTrackerNode::~FeatureTrackerNode()
{

};

void FeatureTrackerNode::featureCallback(const infrastructure_camera_tracker::FeatureSet::ConstPtr& features)
{

};

void FeatureTrackerNode::imageCallback(const sensor_msgs::Image::ConstPtr& image)
{

};