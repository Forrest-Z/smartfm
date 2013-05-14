#include <ros/ros.h>

#include <infrastructure_camera_tracker/TrackedFeatureSet.h>  // subscribes
#include <infrastructure_camera_tracker/ArrivalSet.h>         // publishes
#include <infrastructure_camera_tracker/TrackSet.h>           // publishes

#include <vector>
#include <infrastructure_camera_tracker/TrackBuilder.hpp>

class ArrivalEstimatorNode
{
  public:
    ArrivalEstimatorNode();
    ~ArrivalEstimatorNode();
  private:
    ros::NodeHandle node_handle;
    ros::Publisher  arrival_publisher;
    ros::Publisher  track_publisher;
    ros::Subscriber feature_subscriber;
    std_msgs::Header last_header;
    TrackBuilder track_builder;

    void featureCallback(const infrastructure_camera_tracker::TrackedFeatureSet::ConstPtr& msg);

    void publishArrivals(void) const;

    void publishTracks(void) const;
};