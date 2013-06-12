#include <ros/ros.h>                                          // ROS

#include <sensor_msgs/Image.h>                                // Messages
#include <infrastructure_camera_tracker/TrackSet.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>

#include <string>

namespace ict = infrastructure_camera_tracker;

class TrackViewerNode
{
  public:
    TrackViewerNode();
    ~TrackViewerNode();

  private:
    ros::NodeHandle node_handle;

    image_transport::ImageTransport img_transport;
    image_transport::Subscriber img_subscriber;
    ros::Subscriber trackset_subscriber;

    static std::vector<cv::Scalar> colormap;

    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, ict::TrackSet> sync_policy;

    // message_filters::Synchronizer<sync_policy> synchronizer;

    std::string overlay_window_name;
    std::string track_window_name;

    cv::Mat accumulated_tracks;
    cv::Mat latest_frame;

    // void callback(  const sensor_msgs::Image::ConstPtr& image,
                    // const ict::TrackSet::ConstPtr& trackset);

    void imageCallback(const sensor_msgs::Image::ConstPtr& image);

    void trackSetCallback(const ict::TrackSet::ConstPtr& trackset);
};
