#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <cmath>
#include "sensing_on_road/line_piece.h"
#include "sensing_on_road/scan_segment.h"
#include "sensing_on_road/segments_one_batch.h"

#ifndef SENSING_ON_ROAD_LASER_EXTRACTION
#define SENSING_ON_ROAD_LASER_EXTRACTION

namespace sensing_on_road{

/*
 **************************************************************
 **following two classes have been represented as .msg files.
 **************************************************************
class scan_segment
{
	public:

	//way to refer to raw information;
	std::vector<unsigned int> 	beam_serial;
	//processed features to use;
	geometry_msgs::Point32		segment_centroid;
	float 							centroid_range_to_laser;
	float								normalized_dimension;
	float								internal_std;
	float								mean_average_deviation;
};	

class segments_one_batch
{
	public:
	Header header;
	std::vector<scan_segment> segments;
};
*/

class laser_extraction
{
	public:
	laser_extraction();
	~laser_extraction();

	private:
    ros::NodeHandle nh_, private_nh_;	
    std::string     ldmrs_single_id_, odom_frame_id_;
	laser_geometry::LaserProjection projector_;   
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::TransformListener tf_; 
    tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;

	float 												angle_resolution_;	
	sensing_on_road::segments_one_batch			extracted_segments_laser_;
	sensing_on_road::segments_one_batch			extracted_segments_odom_;
	sensor_msgs::LaserScan							laser_scan_;
	
	//segmentation and data association are conducted in different frames;
	sensor_msgs::PointCloud 						laser_cloud_laser_;
	sensor_msgs::PointCloud 						laser_cloud_odom_;
	sensor_msgs::PointCloud 						segment_centroids_;

	ros::Publisher 									laser_pub_;   
	ros::Publisher 									segment_border_pub_;  
	ros::Publisher 									segment_processed_pub_; 
   ros::Publisher 									perpendicular_pub_;
   ros::Publisher 									segment_centroids_pub_;
	void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void segmentation();
	void segments_processing();
	void single_segment_processing(sensing_on_road::scan_segment &segment_para);
   void line_piece_calculation(sensing_on_road::line_piece &line_piece_para);
};
};
#endif

