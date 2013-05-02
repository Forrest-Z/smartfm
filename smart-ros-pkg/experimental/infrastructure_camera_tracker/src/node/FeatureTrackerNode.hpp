#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <infrastructure_camera_tracker/FeatureTracker.hpp>

#include <std_msgs/Header.h>

enum tracker_state { sample, track };

class FeatureTrackerNode
{
  public:
    FeatureTrackerNode();
    ~FeatureTrackerNode();

  private:
    ros::NodeHandle node_handle;

    FeatureTracker tracker;

    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_subscriber;

    ros::Publisher track_publisher;

    tracker_state state;

    std_msgs::Header header;

    double time_last_restart;

    void imageCallback(const sensor_msgs::Image::ConstPtr& image);
    void publishFeatures(bool new_set);
};