#include "FeatureViewerNode.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

FeatureViewerNode::FeatureViewerNode(): node_handle("~"),
                                        image_subscriber(node_handle, "image", 1),
                                        feature_subscriber(node_handle, "features", 1),
                                        synchronizer(sync_policy(1), image_subscriber, feature_subscriber),
                                        window_name("feature viewer")
{
  // bind the callback and tell boost::bind() to forward the first and second arguments it receives
  boost::function<void (sensor_msgs::Image::ConstPtr&, infrastructure_camera_tracker::Features&)> callback( boost::bind( &FeatureViewerNode::callback, this, _1, _2 ) );
  //synchronizer.registerCallback( &callback );

  //cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
};

FeatureViewerNode::~FeatureViewerNode()
{
  // close window
};

void FeatureViewerNode::callback( sensor_msgs::Image::ConstPtr& image,  infrastructure_camera_tracker::Features& features)
{
  //ROS_INFO("received image/features pair\n");
  //cv::Mat frame = cv_bridge::toCvCopy(image, "bgr8")->image;
  //cv::imshow(window_name, frame);

  //cv::waitKey(10);
};
