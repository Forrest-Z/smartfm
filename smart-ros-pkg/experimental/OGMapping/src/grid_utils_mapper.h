#ifndef GOLFCAR_PERCEPTION_GRID_UTILS_MAPPER_H
#define GOLFCAR_PERCEPTION_GRID_UTILS_MAPPER_H

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <occupancy_grid_utils/ray_tracer.h>
#include <laser_geometry/laser_geometry.h>

namespace golfcar_perception{

	using namespace std;
	namespace gm=geometry_msgs;
	namespace sm=sensor_msgs;
	namespace nm=nav_msgs;
	namespace gu=occupancy_grid_utils;

	typedef boost::shared_ptr<gu::LocalizedCloud> CloudPtr;
	typedef boost::shared_ptr<gu::LocalizedCloud const> CloudConstPtr;

	class grid_utils_mapper
	{
		
		public:
		grid_utils_mapper();

		private:
		ros::NodeHandle nh_, private_nh_;
		boost::recursive_mutex configuration_mutex_;
		void scanCallback (const sm::LaserScan& scan);

		nm::MapMetaData info_;

		//double origin_x_, origin_y_, origin_yaw_;
		//unsigned int xsize_, ysize_; double resolution_;

		tf::TransformListener tf_;
		ros::Subscriber scan_sub_;
		ros::Publisher grid_pub_;

	};
};

#endif
