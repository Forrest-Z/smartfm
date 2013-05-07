#include <ros/ros.h>

#include <infrastructure_camera_tracker/TrackedFeatureSet.h>  // subscribes
#include <infrastructure_camera_tracker/TrackSet.h>           // publishes

#include <vector>
#include <infrastructure_camera_tracker/Track.hpp>

class TrackBuilderNode
{
  public:
    TrackBuilderNode();
    ~TrackBuilderNode();
  private:
    ros::NodeHandle node_handle;
    ros::Publisher  track_publisher;
    ros::Subscriber feature_subscriber;

    std::vector<Track> trackset;

    void featureCallback(const infrastructure_camera_tracker::TrackSet::ConstPtr& msg);
};