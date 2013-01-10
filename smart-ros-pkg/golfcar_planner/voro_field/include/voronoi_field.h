#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8MultiArray.h>
#include <dynamicvoronoi.h>

using namespace std;

class VoroField
{
public:
		VoroField();
		~VoroField();

		nav_msgs::OccupancyGrid map_pub;
		float resolution ;
		int height;
		int width;
		bool **gridCell;

		ros::NodeHandle nh;
	    ros::Subscriber map_sub;
	    ros::Publisher cost_map_pub;

	    void map_CB(const nav_msgs::OccupancyGrid map_grid);
	   // void cost_map_publish();
	    void voro_field_update();
};

