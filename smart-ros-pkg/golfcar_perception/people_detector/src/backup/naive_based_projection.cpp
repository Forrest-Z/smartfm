#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

// Here it says tf is too old.

  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud laser_cloud;
  sensor_msgs::PointCloud laser_cloud_transformed;


void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  projector_.projectLaser(*scan_in, laser_cloud);
}

void transformPose(const tf::TransformListener& listener)
{
	laser_cloud.header.frame_id = "sick_laser";
	laser_cloud_transformed.header.frame_id = "base_link";
  	listener.transformPointCloud("base_link", laser_cloud,laser_cloud_transformed);  // Pay attention here to listener.transformPointCloud
}

void Pubtopic(const ros::Publisher& laser_pub)
{
	  laser_pub.publish(laser_cloud_transformed);
}

int main(int argc, char** argv){
	
  ros::init(argc, argv, "naive_based_projection");

  ros::NodeHandle q; 
  ros::Subscriber sub = q.subscribe("sick_scan", 2, scanCallback); // Here sick_scan is the name of published topic from sick laser.

  ros::NodeHandle n;
  tf::TransformListener listener(ros::Duration(10));
  ros::Timer timer1 = n.createTimer(ros::Duration(0.1), boost::bind(&transformPose, boost::ref(listener)));//Here ros::Duration need to be changed later. 
  
  ros::NodeHandle m;
  ros::Publisher laser_pub = m.advertise<sensor_msgs::PointCloud>("laser_cloud", 2);
  ros::Timer timer2 = m.createTimer(ros::Duration(0.1), boost::bind(&Pubtopic, boost::ref(laser_pub)));

  ros::spin();
  
   return 0;
}
