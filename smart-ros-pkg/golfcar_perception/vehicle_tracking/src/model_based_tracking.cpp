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
	void updateScanImages (const sensor_msgs::LaserScan::ConstPtr& scan_in);

	int temporal_window_, beam_number_;
	cv::Mat scan_state_, scan_reading_, expected_state_, expected_reading_;
	std::vector<uchar> scan_state_vec_, expected_state_vec_;
	std::vector<float> scan_reading_vec_, expected_reading_vec_;
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
}

void model_based_tracking::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	ROS_INFO("scan callback");
	sensor_msgs::PointCloud baselink_cloud;
	try{projector_.transformLaserScanToPointCloud(laser_frame_id_, *scan_in, baselink_cloud, tf_);}
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

	updateScanImages(scan_in);
}

void model_based_tracking::updateScanImages (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	ROS_INFO("updateScanImages");

	for(size_t i=0; i<scan_in->ranges.size(); i++)
	{
		if(scan_in->ranges[i]<scan_in->range_min || scan_in->ranges[i]>scan_in->range_max)
		{
			scan_state_vec_.push_back(0);
		}
		else
		{
			scan_state_vec_.push_back(255);
		}

		scan_reading_vec_.push_back(scan_in->ranges[i]);
	}

	if(scan_state_vec_.size()>(size_t)temporal_window_*beam_number_)
	{
		scan_state_vec_.erase(scan_state_vec_.begin(), scan_state_vec_.begin()+beam_number_);
		scan_reading_vec_.erase(scan_reading_vec_.begin(), scan_reading_vec_.begin()+beam_number_);
	}

	std::cout<<temporal_window_<<","<<beam_number_<<","<<scan_state_vec_.size()<<endl;

	uchar scan_state_data[scan_state_vec_.size()];
	float scan_reading_data[scan_state_vec_.size()];
	for(size_t i=0; i<scan_state_vec_.size(); i++)
	{
		scan_state_data[i]=scan_state_vec_[i];
		scan_reading_data[i]=scan_reading_vec_[i]/50.0;
	}
	scan_state_  =	cv::Mat(temporal_window_, beam_number_, CV_8UC1);
	scan_state_.data = scan_state_data;
	scan_reading_ =	cv::Mat(temporal_window_, beam_number_, CV_32FC1, scan_reading_data);


	cv::imshow("scan_state", scan_state_);
	cv::imshow("scan_reading", scan_reading_);

	cv::waitKey(3);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "model_based_tracking_node");
	model_based_tracking model_based_tracking_node;
	ros::spin();
	return (0);
}
