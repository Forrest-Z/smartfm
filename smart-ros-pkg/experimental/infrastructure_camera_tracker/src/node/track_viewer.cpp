#include <ros/ros.h>
#include "TrackViewerNode.hpp" // your code is in another file, Mario!

int main(int argc, char** argv)
{
  ros::init(argc, argv, "track_viewer");

  ROS_INFO("Starting track viewer node.");

  TrackViewerNode node;

  ros::spin();

  return(0);
}