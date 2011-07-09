#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>

// Thread suppport
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

//laser geometery
#include <laser_geometry/laser_geometry.h>

namespace people_detector{
	/**
   * @class people_detect
   * @brief Detect people by comparing with a base map to eliminate known static object
   * thereby select the possible candidates from this segmented data
   */
   
   class people_detect{
	   public:
	   
	   people_detect();
	   
	   ~people_detect();
	   
	   private:
	   
	   void initMap(const nav_msgs::OccupancyGrid& map);
	   
	   void drawMap(std::vector<unsigned int> &cells);
	   ros::Subscriber map_sub_;
	   boost::recursive_mutex map_data_lock_;
	   std::vector<unsigned int> input_data_;
	   unsigned int map_width_, map_height_;
	   
	   //laser geometry
		laser_geometry::LaserProjection projector_;
		sensor_msgs::PointCloud laser_cloud;
		sensor_msgs::PointCloud laser_cloud_transformed;
		
		void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
		tf::TransformListener listener;
		ros::Publisher laser_pub;
		ros::Subscriber laser_sub;
   };
};
