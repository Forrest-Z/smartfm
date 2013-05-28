#include <ros/ros.h>
#include "FeatureTrackerNode.hpp" // your code is in another file, Mario!

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_tracker");

  ROS_INFO("Starting feature tracker node.");

  FeatureTrackerNode node;

  ros::spin();

  return(0);
}
