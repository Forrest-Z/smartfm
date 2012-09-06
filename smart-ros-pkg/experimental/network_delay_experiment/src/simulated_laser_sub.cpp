/*
 * simulated_laser_sub.cpp
 *
 *  Created on: Sep 6, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void scanCallback(sensor_msgs::LaserScanConstPtr scan)
{
    ros::WallTime walltime = ros::WallTime::now();
    ros::Time time_now = ros::Time(walltime.sec, walltime.nsec);
    std::cout<<"Scan seq="<<scan->header.seq<<" Delay="<<(time_now - scan->header.stamp).toSec()<<std::endl;
}

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "simulated_laser_sub");
    ros::NodeHandle n;
    ros::Subscriber laser_sub;
    laser_sub = n.subscribe<sensor_msgs::LaserScan>("simulated_scan", 10, &scanCallback);
    ros::spin();

}
