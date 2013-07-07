#include <ros/ros.h>
#include <diagnostic_updater/publisher.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <cmath>

using namespace std;
using namespace ros;
using namespace tf;


typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

class laser_diff
{

public:
	laser_diff();

private:

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	std::string base_frame_id_;
	std::string odom_frame_id_;
	double speed_limit_;
	double time_diff_;

	laser_geometry::LaserProjection                         projector_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     laser_sub_;
	tf::TransformListener tf_;
	tf::MessageFilter<sensor_msgs::LaserScan>               *tf_filter_;

	//Main Part 0: subscribe to laser topic and do transformation
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void scanProcessing();
	sensor_msgs::LaserScan                                  laser_scan_;
	sensor_msgs::PointCloud                                 current_laser_cloud_;
	geometry_msgs::PoseStamped								current_laser_pose_;


	vector<sensor_msgs::PointCloud>                         laser_cloud_vector_;
	//geometry_msgs::PoseStamped								last_laser_pose_;
	sensor_msgs::PointCloud                                 moving_cloud_;
	ros::Publisher                              			moving_cloud_pub_;
	ros::Publisher                              			last_laser_pub_;
};

laser_diff::laser_diff()
: private_nh_("~")
{
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	moving_cloud_pub_   =   nh_.advertise<sensor_msgs::PointCloud>("moving_cloud", 2);
	last_laser_pub_  =   nh_.advertise<sensor_msgs::PointCloud>("oldest_laser", 2);

	laser_sub_.subscribe(nh_, "/sickldmrs/scan1", 2);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, odom_frame_id_, 10);
	tf_filter_->registerCallback(boost::bind(&laser_diff::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.05));

	private_nh_.param("speed_limit",      speed_limit_,     1.0);
}

void laser_diff::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	laser_scan_ = *scan_in;
	std::string laser_frame_id = scan_in->header.frame_id;
	try{projector_.transformLaserScanToPointCloud(odom_frame_id_, *scan_in, current_laser_cloud_, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
	geometry_msgs::PoseStamped ident;
	ident.header = scan_in->header;
	ident.pose.position.x=0;
	ident.pose.position.y=0;
	ident.pose.position.z=0;
	ident.pose.orientation.x=1;
	ident.pose.orientation.y=0;
	ident.pose.orientation.z=0;
	ident.pose.orientation.w=0;
	try
	{
		this->tf_.transformPose(odom_frame_id_, ident, current_laser_pose_);
	}
	catch(tf::TransformException e)
	{
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return;
	}
	laser_cloud_vector_.push_back(current_laser_cloud_);

	time_diff_ = laser_cloud_vector_.back().header.stamp.toSec()-laser_cloud_vector_.front().header.stamp.toSec();
	if(time_diff_ < 0.1) return;

	laser_diff::scanProcessing();

}

void laser_diff::scanProcessing()
{
	ROS_INFO("scan_process");
	moving_cloud_.header = current_laser_cloud_.header;
	for(size_t i=0; i<current_laser_cloud_.points.size();i++)
	{
		//ROS_INFO("total size %ld, %ld", current_laser_cloud_.points.size(), i);
		double angle_tmp = atan2(current_laser_cloud_.points[i].y-current_laser_pose_.pose.position.y, current_laser_cloud_.points[i].x-current_laser_pose_.pose.position.x);

		for(size_t j=0; j<laser_cloud_vector_.front().points.size();j++)
		{
			double angle_tmpp = atan2(laser_cloud_vector_.front().points[j].y-current_laser_pose_.pose.position.y, laser_cloud_vector_.front().points[j].x-current_laser_pose_.pose.position.x);

			double angle_diff = fabs(angle_tmpp-angle_tmp);
			//ROS_INFO("angle_tmp %3f, angle_tmpp %3f, angle_diff %3f", angle_tmp, angle_tmpp, angle_diff);

			if(angle_diff <  0.7*fabs(laser_scan_.angle_increment))
			{
				double distance_tmp = sqrt((current_laser_cloud_.points[i].y-laser_cloud_vector_.front().points[j].y)*(current_laser_cloud_.points[i].y-laser_cloud_vector_.front().points[j].y)+
							(current_laser_cloud_.points[i].y-laser_cloud_vector_.front().points[j].y)*(current_laser_cloud_.points[i].y-laser_cloud_vector_.front().points[j].y));

				//ROS_INFO("distance_tmp %3f", distance_tmp);

				if(distance_tmp/time_diff_ > speed_limit_)
				{
					moving_cloud_.points.push_back(current_laser_cloud_.points[i]);
					break;
				}
			}
		}
	}

	ROS_INFO("scan_process over");
	moving_cloud_pub_.publish(moving_cloud_);
	moving_cloud_.points.clear();

	sensor_msgs::PointCloud last_cloud_show;
	last_cloud_show = laser_cloud_vector_.front();
	last_cloud_show.header= moving_cloud_.header;
	last_laser_pub_.publish(last_cloud_show);
	laser_cloud_vector_.erase(laser_cloud_vector_.begin());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_diff_node");
	laser_diff laser_diff_node;
	ros::spin();
	return (0);
}
