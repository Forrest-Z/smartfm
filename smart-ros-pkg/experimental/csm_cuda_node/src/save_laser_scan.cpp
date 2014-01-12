/*
 * laser_maxrange_republish.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: golfcar
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "assert.h"
#include "save_laser_scan.h"
ros::Publisher *laser_pub_;


string folder_;
void scanCallback(const sensor_msgs::LaserScanConstPtr msg)
{
    sensor_msgs::LaserScan ls = *msg;
    stringstream ss;
    ss<<folder_;
    char folderEnd = folder_[folder_.length()-1];
    if( folderEnd != '/')
      ss<<"/";
    ss<<msg->header.seq<<".laser_data";
    writeROSMsg<sensor_msgs::LaserScan>(ls, ss.str());
    //sensor_msgs::LaserScan ls2 = readROSFile<sensor_msgs::LaserScan>(ss.str());
    //laser_pub_->publish(ls2);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  
  priv_nh.param("folder", folder_, string(""));
  ros::Publisher laser_pub= n.advertise<sensor_msgs::LaserScan>("scan_out",1);
  laser_pub_ = &laser_pub;
  ros::Subscriber sub = n.subscribe("scan", 1, scanCallback);
  ros::spin();

  return 0;
}
