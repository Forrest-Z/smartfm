/*
 * road_surface_removal.cpp
 *
 *  Created on: Jul 3, 2012
 *      Author: demian
 */

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <robust_road_detection/curb_details.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace std;

class RoadSurfaceRemoval
{
	ros::NodeHandle nh_;
	message_filters::Subscriber<sensor_msgs::PointCloud> laser_cloud_sub_;
	message_filters::Subscriber<robust_road_detection::curb_details> curb_details_sub_;
	ros::Publisher laser_pub_, laser_curb_pub_;
	int last_leftcurb_pt_, last_rightcurb_pt_;
	int buffer_;
	void update_curblines(robust_road_detection::curb_details& details)
	{
		//update the last leftcurb with -1 with last known id
		cout<<"Received: "<<details.left_pt_id<<" "<<details.right_pt_id<<endl;
        if(details.left_pt_id != -1 && details.right_pt_id != -1) assert(details.right_pt_id < details.left_pt_id);
		if(details.left_pt_id != -1) last_leftcurb_pt_ = details.left_pt_id - buffer_;
		if(details.right_pt_id != -1) last_rightcurb_pt_ = details.right_pt_id + buffer_;
	}

	bool filter_laser(sensor_msgs::PointCloud& laser)
	{
		cout<<laser.points.size()<<endl;
		if(last_leftcurb_pt_==-1 || last_rightcurb_pt_ == -1) return false;
		int begin, end;
		if(last_leftcurb_pt_<last_rightcurb_pt_) { begin = last_leftcurb_pt_; end = last_rightcurb_pt_;}
		else { end = last_leftcurb_pt_; begin = last_rightcurb_pt_;}
		laser.points.erase(laser.points.begin()+begin, laser.points.begin()+end);
		return true;
	}

	void filter_and_publish_curb(sensor_msgs::PointCloudConstPtr laser, robust_road_detection::curb_detailsConstPtr details)
	{
		sensor_msgs::PointCloud curb_pt;
		if(details->left_pt_id !=-1)
			curb_pt.points.push_back(laser->points[details->left_pt_id]);
		if(details->right_pt_id != -1)
			curb_pt.points.push_back(laser->points[details->right_pt_id]);
		curb_pt.header = laser->header;
		sensor_msgs::PointCloud2 curb_pcl;
		sensor_msgs::convertPointCloudToPointCloud2(curb_pt, curb_pcl);
		laser_curb_pub_.publish(curb_pcl);
	}

	void syncCallback(sensor_msgs::PointCloudConstPtr laser, robust_road_detection::curb_detailsConstPtr details)
	{
		cout<<last_leftcurb_pt_<<" "<<last_rightcurb_pt_<<endl;
		robust_road_detection::curb_details details_copy = *details;
		update_curblines(details_copy);
		filter_and_publish_curb(laser, details);
		//remove and publish the laser with road surface removed
		sensor_msgs::PointCloud laser_filtered = *laser;
		//remove and publish the laser with only curb points
		//disregard the status of the filter. This is to correct
		//initialization problem where there could be no curb
		filter_laser(laser_filtered);
		sensor_msgs::PointCloud2 laser_filtered2;
		sensor_msgs::convertPointCloudToPointCloud2(laser_filtered, laser_filtered2);
		laser_pub_.publish(laser_filtered2);

	}

public:
	RoadSurfaceRemoval() : last_leftcurb_pt_(-1), last_rightcurb_pt_(-1), buffer_(10)
	{
		laser_cloud_sub_.subscribe(nh_, "laser_cloud_laser", 20);
		curb_details_sub_.subscribe(nh_, "curb_details", 20);
		laser_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("laser_surface_removed", 10);
		laser_curb_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("laser_pure_curb", 10);
		message_filters::TimeSynchronizer<sensor_msgs::PointCloud, robust_road_detection::curb_details> sync(laser_cloud_sub_, curb_details_sub_, 20);
		sync.registerCallback(boost::bind(&RoadSurfaceRemoval::syncCallback, this, _1, _2));
		ros::spin();
	}
};

int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "road_surface_removal");
	RoadSurfaceRemoval rsr;
	return 0;
}
