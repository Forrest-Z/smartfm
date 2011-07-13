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

class local_map
{
    public:
        unsigned char *map;
        void map_init();
        void map_free();
        void on_sick(const sensor_msgs::LaserScan::ConstPtr *msg);

};
