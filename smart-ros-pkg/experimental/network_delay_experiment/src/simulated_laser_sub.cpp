/*
 * simulated_laser_sub.cpp
 *
 *  Created on: Sep 6, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <network_delay_experiment/delay.h>

ros::Publisher *delay_pub;

void scanCallback(sensor_msgs::LaserScanConstPtr scan)
{
    ros::WallTime walltime = ros::WallTime::now();
    ros::Time time_now = ros::Time(walltime.sec, walltime.nsec);
//    std::cout<<"Scan seq="<<scan->header.seq<<" Delay="<<(time_now - scan->header.stamp).toSec()<<std::endl;
    network_delay_experiment::delay delay;
    delay.seq = scan->header.seq;
    delay.time = (time_now - scan->header.stamp).toSec();
    delay_pub->publish(delay);  
}

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "simulated_laser_sub");
    ros::NodeHandle n;
    ros::Subscriber laser_sub;
    ros::Publisher pub;
    laser_sub = n.subscribe("simulated_scan", 10, &scanCallback);
    pub = n.advertise<network_delay_experiment::delay>("delay", 10);
    delay_pub = &pub;
    ros::spin();

}
