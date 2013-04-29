#include <ros/ros.h>                                          // ROS

#include <sensor_msgs/Image.h>                                // Messages
#include <infrastructure_camera_tracker/Features.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <string>

// documentation

class FeatureViewerNode
{
  public:
    FeatureViewerNode();
    ~FeatureViewerNode();

  private:
    ros::NodeHandle node_handle;

    image_transport::ImageTransport img_transport;
    image_transport::SubscriberFilter img_subscriber;
    message_filters::Subscriber<infrastructure_camera_tracker::Features> feature_subscriber;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, infrastructure_camera_tracker::Features> sync_policy;

    message_filters::Synchronizer<sync_policy> synchronizer;

    std::string window_name;

    void callback(  const sensor_msgs::Image::ConstPtr& image,
                    const infrastructure_camera_tracker::Features::ConstPtr& features);
};
