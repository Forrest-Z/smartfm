#include <ros/ros.h>
#include "FeatureDetectorNode.hpp" // your code is in another file, Mario!

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_detector");

  ROS_INFO("Starting feature detector node.");

  FeatureDetectorNode node;

  ros::spin();

  return(0);
}
