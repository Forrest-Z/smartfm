#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "people_detector/people_rects.h"
#include "people_detector/people_rect.h"
int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<people_detector::people_rects>("chatter", 2);
  ros::Rate loop_rate(40);

  while (ros::ok())
  {
	people_detector::people_rect pr;
	pr.scaled_x = 0;
	pr.scaled_y = 0;
	pr.scaled_width = 640;
	pr.scaled_height = 480;
	people_detector::people_rects prs;
	prs.pr_vector.push_back(pr);
	
    chatter_pub.publish(prs);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}

