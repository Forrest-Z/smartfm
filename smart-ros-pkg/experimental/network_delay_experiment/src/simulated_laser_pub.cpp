/*
 * simulated_laser_pub.cpp
 *
 *  Created on: Sep 6, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ctime>

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "simulated_laser");
    ros::NodeHandle n;
    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("simulated_scan", 10);
    std::string name_space = n.getNamespace();
    ros::Rate loop(75);
    srand (time (NULL));
    sensor_msgs::LaserScan scan;
    scan.angle_increment = M_PI/180*0.5;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.scan_time = 0.0133333336562;
    scan.time_increment= 3.70370362361e-05;
    scan.range_min = 0.0;
    scan.range_max = 81.0;
    while(ros::ok())
    {
        for(double angle = scan.angle_min; angle <= scan.angle_max; angle+=scan.angle_increment)
            scan.ranges.push_back(81.90999603271484f * rand () / (RAND_MAX + 1.0f));
        if(name_space.size() <= 1) name_space.clear();
        scan.header.frame_id = name_space+"/laser";
        ros::WallTime walltime = ros::WallTime::now();
        ros::Time time(walltime.sec, walltime.nsec);
        scan.header.stamp = time;
        laser_pub.publish(scan);
        ros::spinOnce();
        loop.sleep();
    }

}
