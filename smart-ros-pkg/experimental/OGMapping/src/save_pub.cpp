#include "ros/ros.h"
#include "OGMapping/save_map.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_saver_publisher");

  ros::NodeHandle n;
	ros::Publisher savemap_pub = n.advertise<OGMapping::save_map>("/save_Tmap", 1);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    OGMapping::save_map msg;
    msg.save = 1;
	 ROS_INFO("publish /save_Tmap");
    savemap_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
