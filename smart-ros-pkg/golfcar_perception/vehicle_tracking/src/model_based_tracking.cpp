#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <laser_geometry/laser_geometry.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <iostream>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <cmath>

#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include "pcl/kdtree/kdtree_flann.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fmutil/fm_stopwatch.h>

using namespace std;
using namespace ros;

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class model_based_tracking
{

public:
	model_based_tracking();

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	std::string laser_frame_id_;
	std::string base_frame_id_;
	std::string odom_frame_id_;
	std::string map_frame_id_;

	tf::TransformListener tf_;
	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;

	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub_;
	laser_geometry::LaserProjection                         projector_;
	ros::Publisher											beam_pub_, endPt_pub_;

	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void updateScanImages (sensor_msgs::LaserScan &input_scan, sensor_msgs::LaserScan &expected_scan);

	int temporal_window_, beam_number_;
	cv::Mat scan_state_, scan_reading_, expected_state_, expected_reading_;
	std::vector<uchar> scan_state_vec_, expected_state_vec_;
	std::vector<float> scan_reading_vec_, expected_reading_vec_;

	//use this local knowledge to generate expected scan;
	//to maintain this knowledge: (1)updating; (2)down-sampling; (3)erasing;
	PointCloud local_env_pcl_;
	std::vector<sensor_msgs::PointCloud> local_pointcloud_vector_;
	sensor_msgs::PointCloud construct_local_env(sensor_msgs::PointCloud &odom_point_input);

	sensor_msgs::LaserScan generate_expected_scan(sensor_msgs::PointCloud &local_env_pcl);
	sensor_msgs::LaserScan empty_scan_;
	bool empty_scan_initialized_;
};

model_based_tracking::model_based_tracking()
: private_nh_("~")
{
	private_nh_.param("laser_frame_id",     laser_frame_id_,    std::string("front_bottom_lidar"));
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	private_nh_.param("temporal_window",	temporal_window_,   500);
	private_nh_.param("beam_number",		beam_number_,       541);

	laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "front_bottom_scan", 100);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, odom_frame_id_, 100);
	tf_filter_->registerCallback(boost::bind(&model_based_tracking::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.1));

	beam_pub_	= nh_.advertise<geometry_msgs::PolygonStamped>("scan_beam", 2);
	endPt_pub_  = nh_.advertise<sensor_msgs::PointCloud>("endPoints", 2);

	scan_state_vec_.resize(temporal_window_*beam_number_, 0);
	scan_reading_vec_.resize(temporal_window_*beam_number_, 0.0);
	expected_state_vec_.resize(temporal_window_*beam_number_, 0);
	expected_reading_vec_.resize(temporal_window_*beam_number_, 0.0);

	empty_scan_initialized_ = false;
}

void model_based_tracking::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	if(!empty_scan_initialized_)
	{
		empty_scan_ =*scan_in;
		empty_scan_.ranges.resize(beam_number_, 0.0);
		empty_scan_.intensities.clear();
		empty_scan_initialized_ = true;
	}

	sensor_msgs::PointCloud baselink_cloud, odom_cloud;
	try{projector_.transformLaserScanToPointCloud(laser_frame_id_, *scan_in, baselink_cloud, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
	try{projector_.transformLaserScanToPointCloud(odom_frame_id_, *scan_in, odom_cloud, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}

	geometry_msgs::PolygonStamped lidar_beams;
	lidar_beams.header = baselink_cloud.header;
	geometry_msgs::Point32 origin_pt, end_pt;
	origin_pt.x = 0.0; origin_pt.y = 0.0; origin_pt.z = 0.0;
	for(size_t i=0; i<baselink_cloud.points.size(); i++)
	{
		end_pt = baselink_cloud.points[i];
		lidar_beams.polygon.points.push_back(origin_pt);
		lidar_beams.polygon.points.push_back(end_pt);
	}
	lidar_beams.polygon.points.push_back(origin_pt);
	beam_pub_.publish(lidar_beams);
	endPt_pub_.publish(baselink_cloud);

	sensor_msgs::PointCloud local_env_pointcloud = construct_local_env(odom_cloud);
	sensor_msgs::LaserScan expected_scan = generate_expected_scan(local_env_pointcloud);

	sensor_msgs::LaserScan input_scan = *scan_in;
	updateScanImages(input_scan, expected_scan);
}

sensor_msgs::PointCloud model_based_tracking::construct_local_env(sensor_msgs::PointCloud &odom_point_input)
{
	sensor_msgs::PointCloud current_local_env = odom_point_input;
	for(size_t i=0; i<local_pointcloud_vector_.size(); i++)
	{
		for(size_t j=0; j<local_pointcloud_vector_[i].points.size(); j++)
		{
			current_local_env.points.push_back(local_pointcloud_vector_[i].points[j]);
		}
	}

	local_pointcloud_vector_.push_back(odom_point_input);
	if(local_pointcloud_vector_.size()>100)local_pointcloud_vector_.erase(local_pointcloud_vector_.begin());
	return current_local_env;
}

void model_based_tracking::updateScanImages(sensor_msgs::LaserScan &input_scan, sensor_msgs::LaserScan &expected_scan)
{
	for(size_t i=0; i<input_scan.ranges.size(); i++)
	{
		if(input_scan.ranges[i]<input_scan.range_min || input_scan.ranges[i]>input_scan.range_max)scan_state_vec_.push_back(0);
		else scan_state_vec_.push_back(255);
		if(expected_scan.ranges[i]<expected_scan.range_min || expected_scan.ranges[i]>expected_scan.range_max)expected_state_vec_.push_back(0);
		else expected_state_vec_.push_back(255);

		scan_reading_vec_.push_back(input_scan.ranges[i]);
		expected_reading_vec_.push_back(expected_scan.ranges[i]);
	}

	if(scan_state_vec_.size()>(size_t)temporal_window_*beam_number_)
	{
		scan_state_vec_.erase(scan_state_vec_.begin(), scan_state_vec_.begin()+beam_number_);
		scan_reading_vec_.erase(scan_reading_vec_.begin(), scan_reading_vec_.begin()+beam_number_);
		expected_state_vec_.erase(expected_state_vec_.begin(), expected_state_vec_.begin()+beam_number_);
		expected_reading_vec_.erase(expected_reading_vec_.begin(), expected_reading_vec_.begin()+beam_number_);
	}

	//std::cout<<temporal_window_<<","<<beam_number_<<","<<scan_state_vec_.size()<<endl;

	uchar scan_state_data[scan_state_vec_.size()], expected_state_data[expected_state_vec_.size()];
	float scan_reading_data[scan_state_vec_.size()], expected_reading_data_[expected_reading_vec_.size()];
	for(size_t i=0; i<scan_state_vec_.size(); i++)
	{
		scan_state_data[i]=scan_state_vec_[i];
		scan_reading_data[i]=scan_reading_vec_[i]/50.0;
		expected_state_data[i]=expected_state_vec_[i];
		expected_reading_data_[i]=expected_reading_vec_[i]/50.0;
	}
	scan_state_  =	cv::Mat(temporal_window_, beam_number_, CV_8UC1, scan_state_data);
	scan_reading_ =	cv::Mat(temporal_window_, beam_number_, CV_32FC1, scan_reading_data);
	expected_state_  =	cv::Mat(temporal_window_, beam_number_, CV_8UC1, expected_state_data);
	expected_reading_ =	cv::Mat(temporal_window_, beam_number_, CV_32FC1, expected_reading_data_);
	cv::Mat difference_mat;
	cv::absdiff(scan_state_, expected_state_, difference_mat);

	cv::imshow("scan_state", scan_state_);
	cv::imshow("scan_reading", scan_reading_);
	cv::imshow("expected_state", expected_state_);
	cv::imshow("expected_reading", expected_reading_);
	cv::imshow("difference_mat", difference_mat);

	cv::waitKey(3);
}

sensor_msgs::LaserScan model_based_tracking::generate_expected_scan(sensor_msgs::PointCloud &local_env_pointcloud)
{
	sensor_msgs::LaserScan expected_scan = empty_scan_;
	if(local_env_pointcloud.points.size() !=0)
	{
		sensor_msgs::PointCloud env_baselink_pointcloud;
		try{tf_.transformPointCloud(laser_frame_id_, local_env_pointcloud, env_baselink_pointcloud);}
		catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return empty_scan_;}

		//sort all the points into a vector;
		std::vector<float> expected_ranges(beam_number_, 0.0);
		for(size_t i=0; i<env_baselink_pointcloud.points.size(); i++)
		{
			float angle_tmp = atan2f(env_baselink_pointcloud.points[i].y, env_baselink_pointcloud.points[i].x);
			float range_tmp = sqrtf(env_baselink_pointcloud.points[i].x*env_baselink_pointcloud.points[i].x+env_baselink_pointcloud.points[i].y*env_baselink_pointcloud.points[i].y);
			if(angle_tmp < empty_scan_.angle_min || angle_tmp > empty_scan_.angle_max) continue;
			int serial_tmp = floor((angle_tmp-empty_scan_.angle_min)/empty_scan_.angle_increment);
			//cout<<angle_tmp<<","<<empty_scan_.angle_min<<","<<empty_scan_.angle_increment<<","<<serial_tmp<<endl;

			assert(serial_tmp>=0 && serial_tmp<beam_number_);
			if(range_tmp>expected_ranges[serial_tmp]) expected_ranges[serial_tmp] = range_tmp;
		}

		empty_scan_.ranges = expected_ranges;
	}
	return expected_scan;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "model_based_tracking_node");
	model_based_tracking model_based_tracking_node;
	ros::spin();
	return (0);
}
