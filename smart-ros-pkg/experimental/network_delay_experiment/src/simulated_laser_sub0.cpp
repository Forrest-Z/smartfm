/*
 * simulated_laser_sub.cpp
 *
 *  Created on: Sep 6, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <network_delay_experiment/delay.h>
#include <iostream>
ros::Publisher *delay_pub;
using namespace std;
void scanCallback(sensor_msgs::LaserScanConstPtr scan)
{
    ros::WallTime walltime = ros::WallTime::now();
    ros::Time time_now = ros::Time(walltime.sec, walltime.nsec);
//  std::cout<<"Scan seq="<<scan->header.seq<<" Delay="<<(time_now - scan->header.stamp).toSec()<<std::endl;
    network_delay_experiment::delay delay;
    delay.seq = scan->header.seq;
    delay.time_now = time_now;
    delay.scan_time = scan->header.stamp;
    delay.nsec_delay = (time_now - scan->header.stamp).toNSec();
    delay_pub->publish(delay);
    cout<<  (time_now - scan->header.stamp).toSec()<<endl;
}

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "simulated_laser_sub_0");
    ros::NodeHandle n;
    
    ros::Subscriber laser_sub;
    ros::Publisher  pub;
    
    laser_sub = n.subscribe<sensor_msgs::LaserScan>("/golfcart/front_bottom_scan_republish", 10, &scanCallback);

	 pub = n.advertise<network_delay_experiment::delay>("delay", 10);
	 delay_pub = &pub;
	 
    ros::spin();
}
