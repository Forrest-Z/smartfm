#ifndef GOLFCAR_PERCEPTION_OGMAPPING_H
#define GOLFCAR_PERCEPTION_OGMAPPING_H

#include <ros/ros.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <cstring>
#include <libgen.h>
#include <fstream>
#include "ros/console.h"
#include "yaml-cpp/yaml.h"

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include <SDL/SDL_image.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include "map.h"
#include "OGMapping/save_map.h"

namespace golfcar_perception{

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	using namespace std;
	class OGMapper {
		
	public:
	OGMapper();
	~OGMapper();
	
	private:
	ros::NodeHandle nh_, private_nh_;
	string map_frame_;
	tf::TransformListener *tf_;

	message_filters::Subscriber<PointCloud> 	boundary_sub_;
	message_filters::Subscriber<PointCloud>		surface_sub_;
	tf::MessageFilter<PointCloud> 				*surface_filter_;
	tf::MessageFilter<PointCloud>				*boundary_filter_;

	std::string map_file_;
	map_t* map_;
	boost::recursive_mutex configuration_mutex_;

	void mapSave_Callback(const OGMapping::save_map::ConstPtr& save_pointer);
	void mapsave();
	void map_load(const std::string& fname);
	void
	loadMapFromFile(map_t* map,
	                const char* fname, double res, bool negate,
	                double occ_th, double free_th, double* origin);

	inline void grayscale_to_probility(uint8_t & pixel_value, uint8_t & occ_prob);
	inline void probility_to_grayscale(double & occ_prob, uint8_t & pixel_value);
	void freeMapDependentMemory();

	void boundaryCallback(const PointCloud::ConstPtr& boundary_in);
	void surfaceCallback(const PointCloud::ConstPtr& surface_in);
	void map_update(PointCloud& input_pcl, std::string& input_type);

	//probability of p(x|z);
	double initial_occ_;
	double occ_over_occ_, occ_over_free_;

	ros::Subscriber	save_sub_;
	bool init_map_, map_saved_;
	int map_size_x_, map_size_y_;
	void unknownMap(map_t* map, double res, unsigned int size_x, unsigned int size_y, double* origin);
    };
};

#endif
