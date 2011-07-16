#include <iostream>

#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"


#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>


  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud laser_cloud;
  extern tf::TransformListener listener_;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  laser_cloud.header.frame_id = "base_link";
  tf::TransformListener listener_;
  projector_.transformLaserScanToPointCloud("base_link",*scan_in,
          laser_cloud, listener_);
}


void Pubtopic(const ros::Publisher& laser_pub)
{
	  laser_pub.publish(laser_cloud);
}

int main(int argc, char** argv){
	
  ros::init(argc, argv, "laser_projection");
  
  ros::NodeHandle m;
  ros::Publisher laser_pub = m.advertise<sensor_msgs::PointCloud>("laser_cloud", 2);
  ros::Timer timer = m.createTimer(ros::Duration(0.1), boost::bind(&Pubtopic, boost::ref(laser_pub)));


  ros::NodeHandle q; 
  
  ros::Subscriber sub = q.subscribe("sick_scan", 2, scanCallback); // Here sick_scan is the name of published topic from sick laser.

  ros::spin();
  
   return 0;
}
