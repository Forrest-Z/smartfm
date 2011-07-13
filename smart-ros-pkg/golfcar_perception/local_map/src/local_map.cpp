#include <iostream>
using namespace std;

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

// Thread suppport
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

//laser geometery
#include <laser_geometry/laser_geometry.h>

//add message_filter
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#define MAP_RES     (0.05)
#define MAP_WIDTH   (5.0)
#define MAP_HEIGHT  (5.0)


class local_map
{


};
unsigned char *map;

// vehicle location is always (0,0)
void map_init()
{
    map = calloc(sizeof(unsigned char), (int)((MAP_WIDTH*MAP_HEIGHT)/MAP_RES/MAP_RES));;
    
}
void map_delete()
{
    free map;
}

void on_sick(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    cout<<"got message with: " << msg.header.frame_id << endl;
    
    float *angles;
    float *ranges;
    int nranges = msg->ranges.size();
    angles = new float[nranges];
    ranges = new float[nranges];

    float curr_angle = 0;
    for(int i=0; i< nranges; i++)
    {
        ranges[i] = msg->ranges[i];
        angles[i] = curr_angle;
        curr_angle += msg->angle_increment;
    }
    
    // transform to base_link frame here
    
    // 

    free angles;
    free ranges;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_map");

    ros::NodeHandle nh;
    
    ros::Subscriber sick1;
    sick1 = nh.subscribe(nh, "sick_scan", 2, on_sick);
    //ros::Subscriber sick2 = nh.subscribe(nh, "sick_scan2", 2, on_sick);

    ros::spin();

}

