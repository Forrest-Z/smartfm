#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int8MultiArray.h>
#include <dynamicvoronoi.h>
#include <time.h>

using namespace std;

class VoroField
{
	friend class DynamicVoronoi;
public:
		VoroField();
		~VoroField();

		sensor_msgs::PointCloud local_map_pub;
		float resolution;
		int height;
		int width;
		bool **gridCell;

		ros::NodeHandle nh;
	    ros::Subscriber map_sub;
	    ros::Publisher cost_map_pub;

	    void map_CB(const nav_msgs::OccupancyGrid map_grid);
	    void cost_map_publish();
	    void voro_field_update();
};

class Time {
public:
  struct timeval tv;
  Time(){reset();}
  void reset(){
    gettimeofday( &tv, NULL);
  }
  float get_since(){
    struct timeval t2;
    gettimeofday( &t2, NULL);
    return (t2.tv_sec-tv.tv_sec) +
      0.000001*(t2.tv_usec-tv.tv_usec);
  }
};

