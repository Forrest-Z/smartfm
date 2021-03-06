#include "FeatureViewerNode.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

FeatureViewerNode::FeatureViewerNode(): node_handle("~"),
                                        img_transport(node_handle),
                                        img_subscriber(img_transport, "image", 1),
                                        feature_subscriber(node_handle, "features", 1),
                                        synchronizer(sync_policy(1), img_subscriber, feature_subscriber),
                                        window_name("feature viewer")
{
  // bind the callback and tell boost::bind() to forward the first and second arguments it receives
  // boost::function<void (const sensor_msgs::Image::ConstPtr&, const infrastructure_camera_tracker::FeatureSet::ConstPtr&)> sync_callback( boost::bind( &FeatureViewerNode::callback, this, _1, _2 ) );
  // synchronizer.registerCallback( sync_callback );                                                // this does not work

  synchronizer.registerCallback( boost::bind(&FeatureViewerNode::callback, this, _1, _2) );         // but this does

  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
};

FeatureViewerNode::~FeatureViewerNode()
{
  // close window
};

void FeatureViewerNode::callback( const sensor_msgs::Image::ConstPtr& image,
                                  const infrastructure_camera_tracker::FeatureSet::ConstPtr& features)
{
  ROS_INFO("received image/features pair\n");
  cv::Mat frame = cv_bridge::toCvCopy(image, "bgr8")->image;

  // plot features found
  for (unsigned int i = 0; i < features->features.size(); ++i)
  {
    cv::circle(frame, cv::Point2f(features->features[i].position.x, features->features[i].position.y), 1, cv::Scalar(0, 255, 0), -1, CV_AA, 0 );
  }

  // display image
  cv::imshow(window_name, frame);
  cv::waitKey(10);
};
