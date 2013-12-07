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
#include <std_msgs/UInt16.h>
#include <fmutil/fm_filter.h>
#include <pnc_msgs/move_status.h>

using namespace std;
using namespace tf;

class dynamic_safety_zone
{
	public:
    ros::NodeHandle nh_, private_nh_;
    tf::TransformListener tf_;
    tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pc2_filter_;
    fmutil::LowPassFilter vFilter_;
    string base_frame_;
    ros::Subscriber odo_sub_, advised_speed_sub_, vehicle_angle_sub_;
    ros::Publisher  fused_speed_pub_, current_polygon_pub_, inner_zone_pub_, remained_pcl_pub_;
    double control_freq_;
    double offcenter_x_, offcenter_y_;
    double x_buffer_, y_buffer_;
    double x_coeff_, y_coeff_, angle_coeff_;
    double x_threshold_, y_threshold_;

    double current_vel_;
    double current_angle_;
    
    double front_boundary_, side_boundary_, back_boundary_;
    geometry_msgs::PolygonStamped inner_safety_zone_;
    sensor_msgs::PointCloud remained_pointcloud_;

    geometry_msgs::Twist advised_speed_, fused_speed_;
    message_filters::Subscriber<sensor_msgs::LaserScan>			laser_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>		pc2_sub_;
    laser_geometry::LaserProjection projector_;
    
    std_msgs::UInt16 obstacle_status_flag_; 
    ros::Time stop_time_;
    bool stop_time_init_;
    ros::Publisher obstacle_status_pub_;
    double stop_time_threshold_;
    ros::Time filter_time_;
	dynamic_safety_zone()
	{
		ros::NodeHandle private_nh("~");

		private_nh.param("base_frame", base_frame_, string("base_link"));
		private_nh.param("control_freq", control_freq_, 5.0);

		private_nh.param("offcenter_x", offcenter_x_, 1.7);
		private_nh.param("offcenter_y", offcenter_y_, 0.8);
		private_nh.param("back_boundary", back_boundary_, 0.5);

		private_nh.param("x_buffer", x_buffer_, 1.5);
		private_nh.param("y_buffer", y_buffer_, 0.2);
		private_nh.param("x_coeff", x_coeff_, 2.0);
		private_nh.param("y_coeff", y_coeff_, 0.25);
		private_nh.param("angle_coeff", angle_coeff_, 2.0);

		private_nh.param("stop_time_threshold", stop_time_threshold_, 5.0);


		odo_sub_ = nh_.subscribe("odom", 1, &dynamic_safety_zone::odom_callback, this);
		vehicle_angle_sub_ =  nh_.subscribe("move_status", 1, &dynamic_safety_zone::angle_callback, this);
		
		laser_sub_.subscribe(nh_, "front_bottom_scan", 10);
		tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, base_frame_, 10);
		tf_filter_->registerCallback(boost::bind(&dynamic_safety_zone::laserCB, this, _1));
		tf_filter_->setTolerance(ros::Duration(0.05));

		pc2_sub_ .subscribe(nh_, "sickldmrs/cloud", 10);
	    tf_pc2_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pc2_sub_, tf_, base_frame_, 10);
	    tf_pc2_filter_->registerCallback(boost::bind(&dynamic_safety_zone::pclCB, this, _1));
	    tf_pc2_filter_->setTolerance(ros::Duration(0.05));

	    advised_speed_sub_ = nh_.subscribe("cmd_speed", 1, &dynamic_safety_zone::adviseSpeedCB, this);
	    fused_speed_pub_= nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	    current_polygon_pub_  = nh_.advertise<geometry_msgs::PolygonStamped>("dynamic_safety_zoon", 10);
	    inner_zone_pub_  = nh_.advertise<geometry_msgs::PolygonStamped>("inner_safety_zoon", 10);

	    remained_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud>("remained_pcl", 10);
	
	    //advised_speed_.linear.x = 1.0;
	    vFilter_ = fmutil::LowPassFilter(0.1);
	    filter_time_ = ros::Time::now();
		current_vel_ = 0.0;
		current_angle_ = 0.0;
		
		front_boundary_ = offcenter_x_ + x_buffer_ + x_coeff_*current_vel_;
		side_boundary_  = offcenter_y_ + y_buffer_ + y_coeff_*current_vel_  + angle_coeff_*current_angle_*current_vel_;
		geometry_msgs::Point32 corner[4];
		corner[0].x = front_boundary_;
		corner[0].y = -side_boundary_;
		corner[1].x = front_boundary_;
		corner[1].y = side_boundary_;
		corner[2].x = -back_boundary_;
		corner[2].y = side_boundary_;
		corner[3].x = -back_boundary_;
		corner[3].y = -side_boundary_;
		for(int i=0; i<4; i++) inner_safety_zone_.polygon.points.push_back(corner[i]);

		obstacle_status_flag_.data = 255;
		obstacle_status_pub_ = nh_.advertise<std_msgs::UInt16>("voice_id", 10);
		stop_time_init_ = false;
	}

	void odom_callback(const nav_msgs::OdometryConstPtr &odom_in)
	{
		ros::Time time_now = ros::Time::now();
		double dt = (time_now - filter_time_).toSec();
		current_vel_ = fabs(vFilter_.filter_dt(dt, odom_in->twist.twist.linear.x));
		filter_time_ = time_now;
		front_boundary_ = offcenter_x_ + x_buffer_ + x_coeff_*current_vel_;
		side_boundary_  = offcenter_y_ + y_buffer_ + y_coeff_*current_vel_ + angle_coeff_*current_angle_*current_vel_;
		geometry_msgs::PolygonStamped current_safety_zoon;
		current_safety_zoon.header = odom_in->header;
		current_safety_zoon.header.frame_id = base_frame_;
		geometry_msgs::Point32 corner[4];
		corner[0].x = front_boundary_;
		corner[0].y = -side_boundary_;
		corner[1].x = front_boundary_;
		corner[1].y = side_boundary_;
		corner[2].x = -back_boundary_;
		corner[2].y = side_boundary_;
		corner[3].x = -back_boundary_;
		corner[3].y = -side_boundary_;

		for(int i=0; i<4; i++) current_safety_zoon.polygon.points.push_back(corner[i]);
		current_polygon_pub_.publish(current_safety_zoon);

		inner_safety_zone_.header = odom_in->header;
		inner_safety_zone_.header.frame_id = base_frame_;
		inner_zone_pub_.publish(inner_safety_zone_);
	}

    void adviseSpeedCB(const geometry_msgs::TwistConstPtr& speed_in)
    {
    	advised_speed_ = *speed_in;
    }

    void pclCB(const sensor_msgs::PointCloud2ConstPtr& pcl_in)
    {
    	sensor_msgs::PointCloud pcl;
    	sensor_msgs::convertPointCloud2ToPointCloud(*pcl_in, pcl);
    	try{tf_.transformPointCloud(base_frame_, pcl, pcl);}
    	catch(tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}
    	
    	calc_min_speed(pcl);
    	check_obstacle_status();
    }

    void laserCB(const sensor_msgs::LaserScanConstPtr& scan_in)
    {
        sensor_msgs::PointCloud pcl;
        try{projector_.transformLaserScanToPointCloud(base_frame_, *scan_in, pcl, tf_);}
        catch (tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}
        calc_min_speed(pcl);
        check_obstacle_status();
    }

    void calc_min_speed(sensor_msgs::PointCloud &local_points)
    {
		remained_pointcloud_.header = local_points.header;
	    remained_pointcloud_.points.clear();
	    
    	double minimum_speed = advised_speed_.linear.x;
    	for(size_t i=0; i<local_points.points.size(); i++)
    	{
			geometry_msgs::Point32 point_tmp = local_points.points[i];
			//if(point_tmp.x < 1.7 && point_tmp.y < 0.7 && point_tmp.y > -0.7) continue;
			//if point out of current_polygon, ignore;
			if((point_tmp.x < -back_boundary_ || point_tmp.x > front_boundary_) || (point_tmp.y>side_boundary_||point_tmp.y<-side_boundary_)) continue;

			remained_pointcloud_.points.push_back(point_tmp);

			double speed_from_x, speed_from_y, min_speed_tmp;
			speed_from_x = (point_tmp.x - offcenter_x_ - x_buffer_)/x_coeff_;
			if(point_tmp.y<0.0) point_tmp.y = - point_tmp.y;
			speed_from_y = (point_tmp.y - offcenter_y_ - y_buffer_ - angle_coeff_*current_angle_*current_vel_)/y_coeff_;

			//ROS_INFO("(%f, %f), %f, %f", point_tmp.x, point_tmp.y, speed_from_x, speed_from_y);

			if(speed_from_x >0.0 && speed_from_y >0.0)
			{
				min_speed_tmp = speed_from_x > speed_from_y? speed_from_x:speed_from_y;
			}
			else if(speed_from_x >0 && speed_from_y <= 0)
			{
				min_speed_tmp =  speed_from_x;
			}
			else if(speed_from_x <0 && speed_from_y >= 0)
			{
				min_speed_tmp =  speed_from_y;
			}
			else 
			{
				min_speed_tmp = 0.0;
			}

    		if(min_speed_tmp < minimum_speed) minimum_speed = min_speed_tmp;
    	}

		//very important: to avoid sudden acceleration when vehicle is static;
		if(minimum_speed > current_vel_)
		{
            double speed_temp;
		  //double speed_temp = current_vel_ + (minimum_speed-current_vel_)/control_freq_;
            if(minimum_speed-current_vel_ >0.3) speed_temp = current_vel_ + 0.3;
            else speed_temp = current_vel_ + (minimum_speed-current_vel_);
			minimum_speed = speed_temp > 0.6? speed_temp:0.6;
		}
		
    	fused_speed_ = advised_speed_;
    	if(advised_speed_.linear.x > minimum_speed) fused_speed_.linear.x = minimum_speed;
    	fused_speed_pub_.publish(fused_speed_);
		remained_pcl_pub_.publish(remained_pointcloud_);
    }
    
    void check_obstacle_status()
    {
		obstacle_status_flag_.data = 255;
		
		if(remained_pointcloud_.points.size()>0) obstacle_status_flag_.data = 4;
		
		if(remained_pointcloud_.points.size()>0 && current_vel_ < 0.1)
		{
			if(!stop_time_init_){
			  stop_time_init_ = true;
			  stop_time_ = ros::Time::now();
			}
			else
			{
				ros::Time time_now_tmp = ros::Time::now();
				double time_difference_tmp = time_now_tmp.toSec() - stop_time_.toSec();
				//cout<<time_difference_tmp<<endl;
				if(time_difference_tmp>stop_time_threshold_){obstacle_status_flag_.data = 3;}
			}
		}
		else
		{
			stop_time_ = ros::Time::now();
		}
		
		obstacle_status_pub_.publish(obstacle_status_flag_);
	}
	
	void angle_callback(const pnc_msgs::move_statusConstPtr &move_status_in)
	{
	  current_angle_ = fabs(move_status_in->steer_angle);//> 0.0? move_status_in->steer_angle:0.0 ;
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
