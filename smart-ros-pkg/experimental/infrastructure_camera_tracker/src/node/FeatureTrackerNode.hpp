#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <infrastructure_camera_tracker/Feature.h>
#include <infrastructure_camera_tracker/FeatureSet.h>
#include <infrastructure_camera_tracker/Track.h>
#include <infrastructure_camera_tracker/TrackSet.h>

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>

enum tracker_state { initialize, resample, track };

class FeatureTrackerNode
{
  public:
    FeatureTrackerNode();
    ~FeatureTrackerNode();

  private:
    ros::NodeHandle node_handle;

    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_subscriber;
    message_filters::Subscriber<infrastructure_camera_tracker::FeatureSet> feature_subscriber;
    ros::Publisher track_publisher;

    tracker_state state;

    void featureCallback(const infrastructure_camera_tracker::FeatureSet::ConstPtr& features);
    void imageCallback(const sensor_msgs::Image::ConstPtr& image);
};