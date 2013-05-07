#include "TrackBuilderNode.hpp"

TrackBuilderNode::TrackBuilderNode(): node_handle("~")
{
  feature_subscriber = node_handle.subscribe("features", 100, &TrackBuilderNode::featureCallback, this);
  track_publisher = node_handle.advertise<infrastructure_camera_tracker::TrackSet>("tracks", 1000);
};

TrackBuilderNode::~TrackBuilderNode()
{

};

void TrackBuilderNode::featureCallback(const infrastructure_camera_tracker::TrackSet::ConstPtr& msg)
{

};