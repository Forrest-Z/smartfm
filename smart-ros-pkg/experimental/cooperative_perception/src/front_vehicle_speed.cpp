/*
 * vehicles_transform.cpp
 *
 *  Created on: Jul 10, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/LaserScan.h>
#include <fmutil/fm_math.h>
#include <fmutil/fm_filter.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

using std::string;

class FrontVehicleSpeed
{
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
    tf::MessageFilter<geometry_msgs::PoseStamped>       *pose_filter_;
    fmutil::LowPassFilter *filter_;
    ros::NodeHandle *nh_;
    tf::TransformListener *tf_;

    ros::Publisher vehicle_odom_pub_;
    geometry_msgs::PoseStamped previous_pose_;
    bool initialized_;
    void poseCallback(geometry_msgs::PoseStampedConstPtr pose)
    {
        geometry_msgs::PoseStamped pose_out;
        tf_->transformPose("/odom", *pose,  pose_out);

        if(!initialized_)
        {
            previous_pose_ = pose_out;
            initialized_ = true;
        }
        else
        {
            double x_diff = pose_out.pose.position.x - previous_pose_.pose.position.x;
            double y_diff = pose_out.pose.position.y - previous_pose_.pose.position.y;
            double t_diff = (pose_out.header.stamp - previous_pose_.header.stamp).toSec();
            nav_msgs::Odometry odom;
            odom.header = pose_out.header;
            odom.pose.pose = pose_out.pose;
            double speed = (sqrt(x_diff*x_diff + y_diff*y_diff))/t_diff;
            double filtered_speed = filter_->filter(pose_out.header.stamp.toSec(), speed);
            odom.twist.twist.linear.x = filtered_speed;
            vehicle_odom_pub_.publish(odom);
            previous_pose_ = pose_out;
        }
            
    }
public:
    FrontVehicleSpeed(): nh_(new ros::NodeHandle), tf_(new tf::TransformListener), initialized_(false)
    {
        pose_sub_.subscribe(*nh_, "vehicle_pose", 10);
        pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(pose_sub_, *tf_, string("/odom"), 10);
        pose_filter_->registerCallback(boost::bind(&FrontVehicleSpeed::poseCallback, this, _1));
        vehicle_odom_pub_ = nh_->advertise<nav_msgs::Odometry>("vehicle_odom_pose", 10);
        filter_ = new fmutil::LowPassFilter(0.4);
    }
};

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "front_vehicle_speed");
    FrontVehicleSpeed fvs;
    ros::spin();
    return 0;
}
