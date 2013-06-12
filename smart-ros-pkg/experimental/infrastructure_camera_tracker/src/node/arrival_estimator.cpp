#include <ros/ros.h>
#include "ArrivalEstimatorNode.hpp" // your code is in another file, Mario!

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arrival_estimator");

  ROS_INFO("Starting arrival estimator node.");

  ArrivalEstimatorNode node;

  ros::spin();

  return(0);
}
