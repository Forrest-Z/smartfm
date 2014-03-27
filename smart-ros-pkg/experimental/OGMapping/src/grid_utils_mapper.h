#ifndef GOLFCAR_PERCEPTION_GRID_UTILS_MAPPER_H
#define GOLFCAR_PERCEPTION_GRID_UTILS_MAPPER_H

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>

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
	typedef boost::circular_buffer<CloudConstPtr> CloudBuffer;
	class grid_utils_mapper
	{
		
		public:
		grid_utils_mapper();

		private:
		boost::recursive_mutex configuration_mutex_;
		ros::NodeHandle nh_, private_nh_;
		tf::TransformListener tf_;
		tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
		message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub_;
		laser_geometry::LaserProjection                         projector_;
		std::string laser_frame_id_, map_frame_id_;

		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
		nm::MapMetaData info_;
		gu::OverlayClouds overlay_;

		ros::Publisher grid_pub_, collected_cloud_pub_;
		CloudConstPtr last_cloud_;
		int process_interval_;
		int scan_accNum_;
		CloudBuffer clouds_;

	};
};

#endif
