#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>

#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

using namespace std;
using namespace ros;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class DATMO
{

public:
	DATMO();

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber filtered_cloud_sub_;
	ros::Publisher	environment_status_pub_;
	int 			time_window_;
	std::vector<int> moving_status_history_;
	double 			moving_threshold_;
	void pcl_callback (const PointCloudRGB::ConstPtr& pcl_in);
};

DATMO::DATMO()
: private_nh_("~")
{
	//assume your input frequency is 25Hz, and your waiting time is 20 Hz;
	private_nh_.param("time_window",			time_window_,     		50);
	private_nh_.param("moving_threshold",		moving_threshold_,      0.5);
	filtered_cloud_sub_ = nh_.subscribe("filtered_pcl", 10, &DATMO::pcl_callback, this);
	environment_status_pub_  = 	nh_.advertise<geometry_msgs::PointStamped>("environment_status", 10);
}

void DATMO::pcl_callback (const PointCloudRGB::ConstPtr& pcl_in)
{
	int moving_points_number = 0;
	if(pcl_in->points.size()==0) moving_points_number =0;
	else {moving_points_number = (int)pcl_in->points.size();}

	moving_status_history_.push_back(moving_points_number);
	if(moving_status_history_.size()>size_t(time_window_))moving_status_history_.erase(moving_status_history_.begin());

	geometry_msgs::PointStamped current_status;
	current_status.header = pcl_in->header;

	current_status.point.x = 0.0;
	current_status.point.y = 0.0;
	for(size_t i=0; i<moving_status_history_.size(); i++)
	{
		if(moving_status_history_[i]>0)current_status.point.x++;
	}
	current_status.point.x = current_status.point.x/(double)moving_status_history_.size();

	if(current_status.point.x > moving_threshold_) current_status.point.y = 1.0;
	else current_status.point.y = 0.0;

	environment_status_pub_.publish(current_status);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "moving_check");
	DATMO DATMO_node;
	ros::spin();
	return (0);
}
