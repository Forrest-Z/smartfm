#include "TrackBuilderNode.hpp"

TrackBuilderNode::TrackBuilderNode(): node_handle("~")
{
  feature_subscriber = node_handle.subscribe("features", 100, &TrackBuilderNode::featureCallback, this);
  track_publisher = node_handle.advertise<infrastructure_camera_tracker::TrackSet>("tracks", 1000);
};

TrackBuilderNode::~TrackBuilderNode()
{

};

void TrackBuilderNode::featureCallback(const infrastructure_camera_tracker::TrackedFeatureSet::ConstPtr& msg)
{
  //
  last_header = msg->header;

  unsigned int N = msg->features.size();
  std::vector<long unsigned int> ids(N);
  std::vector<cv::Point2f> points(N);
  std::vector<double> timestamps(N);
  std::vector<bool> active(N);

  for (unsigned int i = 0; i < N; ++i)
  {
    ids[i]        = msg->features[i].id;
    points[i]     = cv::Point2f(msg->features[i].position.x, msg->features[i].position.y);
    timestamps[i] = msg->features[i].time;
    active[i]     = msg->features[i].active;
  }

  if (msg->new_set)
  {
    publishTracks();
    track_builder.clear(); // if stitching isn't implemented
    track_builder.add(ids, points, timestamps);
  }
  else
  {
    track_builder.update(ids, points, timestamps, active);
  }

};

void TrackBuilderNode::publishTracks(void) const
{
  namespace ict = infrastructure_camera_tracker;
  ict::TrackSet message;
  // msg/track as vector of features
  // trackset as a vector of msg/track
  std::vector<Track> tracks = track_builder.getTracks();
  std::vector<ict::Track> trackv_msg(tracks.size());

  for(unsigned int i = 0; i < tracks.size(); ++i)
  {
    std::vector<ict::Feature> featurev_msg(tracks[i].size());

    std::vector<cv::Point2f> positions;
    std::vector<double> timestamps;
    long unsigned int id;
    tracks[i].get(positions, timestamps, id);

    for(unsigned int j = 0; j < tracks[i].size(); ++j )
    {
      featurev_msg[j].position.x = positions[j].x;
      featurev_msg[j].position.y = positions[j].y;
      featurev_msg[j].time = timestamps[j];
    }

    trackv_msg[i].track = featurev_msg;
    trackv_msg[i].id = id;
  }

  message.header = last_header;
  message.tracks = trackv_msg;

  track_publisher.publish(message);
};