#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher *pub_;
void callBack(sensor_msgs::LaserScan scan){
  scan.header.stamp = ros::Time::now();
  pub_->publish(scan);
}
int main(int argc, char** argv){
  ros::init(argc, argv, "laser_repub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan_real", 1000, callBack);
  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("Laser/sick_laser", 
					     10);
  pub_ = &pub;
  ros::spin();
  return 0;
}