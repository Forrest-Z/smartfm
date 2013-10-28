#include <ros/ros.h>
#include "TrackBuilderNode.hpp" // your code is in another file, Mario!

int main(int argc, char** argv)
{
  ros::init(argc, argv, "track_builder");

  ROS_INFO("Starting track builder node.");

  TrackBuilderNode node;

  ros::spin();

  return(0);
}
