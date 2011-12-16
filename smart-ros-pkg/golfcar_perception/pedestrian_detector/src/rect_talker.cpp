#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensing_on_road/pedestrian_vision_batch.h"
#include "sensing_on_road/pedestrian_vision.h"
int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<sensing_on_road::pedestrian_vision_batch>("pd_vision_batch", 2);
  ros::Rate loop_rate(40);

  while (ros::ok())
  {
	sensing_on_road::pedestrian_vision pr;
	pr.x = 0;
	pr.y = 0;
	pr.width = 640;
	pr.height = 360;
	sensing_on_road::pedestrian_vision_batch prs;
	prs.pd_vector.push_back(pr);
	
    chatter_pub.publish(prs);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}

