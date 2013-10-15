/*
 * dynamic_safety_zone.h
 *
 *  Created on: OCT 15, 2013
 *      Author: golfcar
 */

#include <string>
#include <cmath>
#include <float.h>
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <boost/bind.hpp>
#include <geometry_msgs/PolygonStamped.h>

using namespace std;
using namespace tf;

class dynamic_safety_zone
{
	public:
    ros::NodeHandle nh_, private_nh_;
    tf::TransformListener tf_;
    tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pc2_filter_;

    string base_frame_;
    ros::Subscriber odo_sub_, advised_speed_sub_;
    ros::Publisher  fused_speed_pub_, current_polygon_pub_;
    double sensory_freq_;
    double offcenter_x_, offcenter_y_;
    double x_buffer_, y_buffer_;
    double x_coeff_, y_coeff_;
    double x_threshold_, y_threshold_;

    double current_vel_;
    double front_boundary_, side_boundary_, back_boundary_;

    geometry_msgs::Twist advised_speed_, fused_speed_;
    message_filters::Subscriber<sensor_msgs::LaserScan>			laser_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>		pc2_sub_;
    laser_geometry::LaserProjection projector_;

	dynamic_safety_zone()
	{
		ros::NodeHandle private_nh("~");

		private_nh.param("base_frame", base_frame_, string("base_link"));
		private_nh.param("sensory_freq", sensory_freq_, 50.0);

		private_nh.param("offcenter_x", offcenter_x_, 1.7);
		private_nh.param("offcenter_y", offcenter_y_, 0.8);
		private_nh.param("back_boundary", back_boundary_, 0.5);

		private_nh.param("x_buffer0", x_buffer_, 0.5);
		private_nh.param("y_buffer0", y_buffer_, 0.2);
		private_nh.param("x_coeff", x_coeff_, 2.0);
		private_nh.param("y_coeff", y_coeff_, 0.5);

		odo_sub_ = nh_.subscribe("/odom", 1, &dynamic_safety_zone::odom_callback, this);

		laser_sub_.subscribe(nh_, "scan", 10);
		tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, base_frame_, 10);
		tf_filter_->registerCallback(boost::bind(&dynamic_safety_zone::laserCB, this, _1));
		tf_filter_->setTolerance(ros::Duration(0.05));

		pc2_sub_ .subscribe(nh_, "sickldmrs/cloud", 10);
	    tf_pc2_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pc2_sub_, tf_, base_frame_, 10);
	    tf_pc2_filter_->registerCallback(boost::bind(&dynamic_safety_zone::pclCB, this, _1));
	    tf_pc2_filter_->setTolerance(ros::Duration(0.05));

	    advised_speed_sub_ = nh_.subscribe("cmd_speed", 1, &dynamic_safety_zone::adviseSpeedCB, this);
	    fused_speed_pub_= nh_.advertise<geometry_msgs::Twist>("fused_cmd_speed", 1);
	    current_polygon_pub_  = nh_.advertise<geometry_msgs::PolygonStamped>("/dynamic_safety_zoon", 10);
	}

	void odom_callback(const nav_msgs::OdometryConstPtr &odom_in)
	{
		current_vel_ = odom_in->twist.twist.linear.x;
		front_boundary_ = offcenter_x_ + x_buffer_ + x_coeff_*current_vel_;
		side_boundary_  = offcenter_y_ + y_buffer_ + y_coeff_*current_vel_;

		geometry_msgs::PolygonStamped current_safety_zoon;
		current_safety_zoon.header = odom_in->header;
		current_safety_zoon.header.frame_id = base_frame_;

		geometry_msgs::Point32 corner[4];
		corner[0].x = front_boundary_;
		corner[0].y = -side_boundary_;
		corner[1].x = front_boundary_;
		corner[1].y = side_boundary_;
		corner[2].x = front_boundary_;
		corner[2].y = -back_boundary_;
		corner[3].x = -front_boundary_;
		corner[3].y = -back_boundary_;

		for(int i=0; i<4; i++) current_safety_zoon.polygon.points.push_back(corner[i]);
		current_polygon_pub_.publish(current_safety_zoon);
	}

    void adviseSpeedCB(const geometry_msgs::TwistConstPtr& speed_in)
    {
    	advised_speed_ = *speed_in;
    }

    void pclCB(const sensor_msgs::PointCloud2ConstPtr& pcl_in)
    {
    	sensor_msgs::PointCloud pcl;
    	sensor_msgs::convertPointCloud2ToPointCloud(*pcl_in, pcl);
    	calc_min_speed(pcl);
    }

    void laserCB(const sensor_msgs::LaserScanConstPtr& scan_in)
    {
        sensor_msgs::PointCloud pcl;
        try{projector_.transformLaserScanToPointCloud(base_frame_, *scan_in, pcl, tf_);}
        catch (tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}
        calc_min_speed(pcl);
    }

    void calc_min_speed(sensor_msgs::PointCloud &local_points)
    {
    	double minimum_speed = DBL_MAX;
    	for(size_t i=0; i<local_points.points.size(); i++)
    	{
    		geometry_msgs::Point32 point_tmp = local_points.points[i];
    		if(point_tmp.x > front_boundary_ && (point_tmp.y>side_boundary_||point_tmp.y<-side_boundary_)) continue;

    		double speed_from_x, speed_from_y, min_speed_tmp;
    		speed_from_x = (point_tmp.x - offcenter_x_ - x_buffer_)/x_coeff_;
    		if(point_tmp.y<0) point_tmp.y = - point_tmp.y;
    		speed_from_y = (point_tmp.y - offcenter_y_ - y_buffer_)/y_coeff_;

    		min_speed_tmp = speed_from_x < speed_from_y? speed_from_x:speed_from_y;
    		if(min_speed_tmp < minimum_speed) minimum_speed = min_speed_tmp;
    	}

    	fused_speed_ = advised_speed_;
    	if(advised_speed_.linear.x > minimum_speed) fused_speed_.linear.x = minimum_speed;
    	fused_speed_pub_.publish(fused_speed_);
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_safety_zone");
    ros::NodeHandle n;
    dynamic_safety_zone safety_zone_node;
    ros::spin();
    return (0);
}
