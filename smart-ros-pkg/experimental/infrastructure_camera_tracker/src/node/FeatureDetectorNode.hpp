#include <infrastructure_camera_tracker/FeatureDetector.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point.h>

class FeatureDetectorNode
{
  public:
    FeatureDetectorNode();
    ~FeatureDetectorNode();

    void imageCallback(const sensor_msgs::Image::ConstPtr& image);

  private:
    ros::NodeHandle node_handle;

    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_subscriber;

    FeatureDetector feature_detector;
    ros::Publisher feature_publisher;

    static const geometry_msgs::Point cvPointToROSPoint(const cv::Point2f& cv_point)
    {
      geometry_msgs::Point p;
      p.x = cv_point.x;
      p.y = cv_point.y;
      return(p);
    };
};
