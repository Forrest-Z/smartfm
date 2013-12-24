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

#include "segmentation/segment-graph.h"

using namespace std;
using namespace ros;

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class DATMO
{

public:
	DATMO();

private:

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	std::string laser_frame_id_;
	std::string base_frame_id_;
	std::string odom_frame_id_;
	std::string map_frame_id_;

	tf::TransformListener tf_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub_;
	laser_geometry::LaserProjection                         projector_;

	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& verti_scan_in);

	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
	size_t 													interval_;
	vector<sensor_msgs::PointCloud>                        	cloud_vector_;
	vector<sensor_msgs::PointCloud>                        	baselink_cloud_vector_;

	vector<sensor_msgs::LaserScan>                        	scan_vector_;
	vector<geometry_msgs::PoseStamped>						laser_pose_vector_;

	geometry_msgs::PoseStamped								laser_pose_current_;
	ros::Publisher                              			collected_cloud_pub_;

	PointCloudRGB 											combined_pcl_;
	void graph_segmentation();
};

DATMO::DATMO()
: private_nh_("~")
{
	private_nh_.param("laser_frame_id",     laser_frame_id_,    std::string("front_bottom_lidar"));
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	private_nh_.param("map_frame_id",       map_frame_id_,      std::string("map"));

	int inverval_tmp;
	private_nh_.param("interval",		    inverval_tmp,       	5);
	interval_ = (size_t) inverval_tmp;

	laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/front_bottom_scan", 100);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, odom_frame_id_, 100);
	tf_filter_->registerCallback(boost::bind(&DATMO::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.1));

	collected_cloud_pub_		=   nh_.advertise<sensor_msgs::PointCloud>("collected_cloud", 2);
}

void DATMO::scanCallback (const sensor_msgs::LaserScan::ConstPtr& verti_scan_in)
{
	ROS_INFO("scan callback %u ", verti_scan_in->header.seq);

	//make a local "baselink" copy to facilitate later feature extraction;
	sensor_msgs::PointCloud baselink_verti_cloud;
	try{projector_.transformLaserScanToPointCloud(laser_frame_id_, *verti_scan_in, baselink_verti_cloud, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
	sensor_msgs::PointCloud verti_cloud;
	//make a global "odom" copy for later moving object extraction;
	try{projector_.transformLaserScanToPointCloud(odom_frame_id_, *verti_scan_in, verti_cloud, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
	//make sure both copies have the same number;
	assert(baselink_verti_cloud.points.size()==verti_cloud.points.size());
	collected_cloud_pub_.publish(baselink_verti_cloud);

	//pay attention to use the intensity value;
	scan_vector_.push_back(*verti_scan_in);
	cloud_vector_.push_back(verti_cloud);
	baselink_cloud_vector_.push_back(baselink_verti_cloud);

	geometry_msgs::PoseStamped ident;
	ident.header = verti_scan_in->header;
	ident.pose.position.x=0;
	ident.pose.position.y=0;
	ident.pose.position.z=0;
	ident.pose.orientation.x=1;
	ident.pose.orientation.y=0;
	ident.pose.orientation.z=0;
	ident.pose.orientation.w=0;
	try
	{
		this->tf_.transformPose(odom_frame_id_, ident, laser_pose_current_);
	}
	catch(tf::TransformException e)
	{
		ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
		return;
	}
	laser_pose_vector_.push_back(laser_pose_current_);

	assert(cloud_vector_.size() == laser_pose_vector_.size());

	if(cloud_vector_.size()==interval_)
	{
		graph_segmentation();
		laser_pose_vector_.erase(laser_pose_vector_.begin(), laser_pose_vector_.begin()+1);
		cloud_vector_.erase(cloud_vector_.begin(), cloud_vector_.begin()+ 1);
		scan_vector_.erase(scan_vector_.begin(), scan_vector_.begin()+ 1);
		baselink_cloud_vector_.erase(baselink_cloud_vector_.begin(), baselink_cloud_vector_.begin()+1);
	}
	ROS_INFO("scan callback finished");
}


void DATMO::graph_segmentation()
{
	combined_pcl_.points.clear();
	combined_pcl_.header = cloud_vector_.back().header;
	for(size_t i=0; i<interval_; i++)
	{
		for(size_t j=0; j<cloud_vector_[i].points.size(); j++)
		{
			pcl::PointXYZRGB pt_tmp;
			pt_tmp.x = cloud_vector_[i].points[j].x;
			pt_tmp.y = cloud_vector_[i].points[j].y;
			pt_tmp.z = cloud_vector_[i].points[j].z;
			combined_pcl_.points.push_back(pt_tmp);
		}
	}

	//1st:construct the data structure from collected pointcloud;
	//to learn the method from "segment_image.h"
	int vertix_num = (int)combined_pcl_.points.size();
	std::vector<edge> edge_vector;



}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "DATMO_node");
	DATMO DATMO_node;
	ros::spin();
	return (0);
}
