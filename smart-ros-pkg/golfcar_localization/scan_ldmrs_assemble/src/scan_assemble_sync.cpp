/*
 * scan_assemble_sync.cpp
 *
 *  Created on: Jan 13, 2012
 *      Author: golfcar
 */

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace sensor_msgs;
using namespace message_filters;



class ldmrs_sync
{
public:
    ldmrs_sync();
private:
    void callback(const LaserScanConstPtr& ls1, const LaserScanConstPtr& ls2);
    ros::Publisher assembled_pub;
};
void ldmrs_sync::callback(const LaserScanConstPtr& ls1, const LaserScanConstPtr& ls2)
{
    assert(ls1->ranges.size() == ls2->ranges.size());
    LaserScan ls = *ls1;
    ls.ranges.clear();
    ls.scan_time *= 2.0;
    ls.time_increment *= 2.0;
    ls.range_max = 15;
    for(unsigned int i=0; i< ls1->ranges.size(); i++)
    {
        double range_tmp;
        if(i%2) range_tmp = ls1->ranges[i];
        else range_tmp = ls2->ranges[i];

        ls.ranges.push_back(range_tmp);
    }

    assembled_pub.publish(ls);

}

ldmrs_sync::ldmrs_sync()
{
    ros::NodeHandle nh;

    assembled_pub = nh.advertise<LaserScan>("/sickldmrs/sync", 1);

    message_filters::Subscriber<LaserScan> laser1_sub(nh, "sickldmrs/scan1", 1);
    message_filters::Subscriber<LaserScan> laser2_sub(nh, "sickldmrs/scan2", 1);
    TimeSynchronizer<LaserScan, LaserScan> sync(laser1_sub, laser2_sub, 10);
    sync.registerCallback(boost::bind(&ldmrs_sync::callback,this, _1, _2));

    ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");
  ldmrs_sync ldmrsSync;


  return 0;
}
