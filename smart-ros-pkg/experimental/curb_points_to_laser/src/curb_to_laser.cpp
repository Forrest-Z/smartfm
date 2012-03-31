/*
 * curb_point_to_laser.cpp
 *
 *  Created on: Mar 25, 2012
 *      Author: golfcar
 */

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <laser_geometry/laser_geometry.h>

using namespace std;

class curb_to_laser
{
public:
    curb_to_laser();

private:
    ros::NodeHandle n_;
    ros::Timer timer_;
    message_filters::Subscriber<sensor_msgs::PointCloud> left_curb_sub_, right_curb_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
    ros::Publisher curb_points_pub_, curb_laser_pub_, curb_points2_pub_;
    sensor_msgs::PointCloud curb_points_;

    int max_curb_points_;
    string target_frame_;

    tf::TransformListener tf_;
    tf::MessageFilter<sensor_msgs::PointCloud> *left_curb_filter_, *right_curb_filter_;
    tf::MessageFilter<sensor_msgs::LaserScan> *laser_scan_filter_;
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud laser_cloud_;
    void leftCurbCallback(sensor_msgs::PointCloudConstPtr left_pc);
    void rightCurbCallback(sensor_msgs::PointCloudConstPtr right_pc);
    void addCurbPoints(sensor_msgs::PointCloudConstPtr pc);
    void publishCurb(const ros::TimerEvent& event);
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    void pointcloudsToLaser(PointCloud& input_pc, sensor_msgs::LaserScan& output);
    void scanCallback(const sensor_msgs::LaserScanConstPtr scan_in);
    void curbPointsDistanceThreshold(sensor_msgs::PointCloud& curb_pts, double distance);

    double curb_dist_, scan_dist_;
};

curb_to_laser::curb_to_laser() : tf_()
{
    target_frame_ = "/odom";
    timer_ = n_.createTimer(ros::Duration(0.1), &curb_to_laser::publishCurb, this);
    left_curb_sub_.subscribe(n_, "left_curbline_pcl", 10);
    left_curb_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(left_curb_sub_, tf_, target_frame_, 10);
    left_curb_filter_->registerCallback(boost::bind(&curb_to_laser::leftCurbCallback, this, _1));

    right_curb_sub_.subscribe(n_, "right_curbline_pcl", 10);
    right_curb_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(right_curb_sub_, tf_, target_frame_, 10);
    right_curb_filter_->registerCallback(boost::bind(&curb_to_laser::rightCurbCallback, this, _1));

    laser_scan_sub_.subscribe(n_, "scan", 10);
    laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, tf_, "base_link", 10);
    laser_scan_filter_->registerCallback(boost::bind(&curb_to_laser::scanCallback, this, _1));

    curb_points_pub_ = n_.advertise<sensor_msgs::PointCloud>("curb_points", 10);
    curb_laser_pub_ = n_.advertise<sensor_msgs::LaserScan>("curb_laser", 10);
    curb_points2_pub_ = n_.advertise<sensor_msgs::PointCloud2>("curb_points2", 10);

    max_curb_points_ = 300;
    curb_dist_ = 8.0;
    scan_dist_ = 30.0;
    ros::spin();
}

void curb_to_laser::scanCallback(const sensor_msgs::LaserScanConstPtr scan_in)
{
    sensor_msgs::LaserScan scan_copy = *scan_in;
    scan_copy.range_max = scan_dist_;
    sensor_msgs::PointCloud laser_cloud;
    try{
    projector_.transformLaserScanToPointCloud("base_link", scan_copy,
                                              laser_cloud, tf_);
    }
    catch (tf::TransformException& e){
        ROS_DEBUG("%s",e.what());
        return;
    }
    /*for(size_t i=0; i<laser_cloud.points.size();)
    {
        if(laser_cloud.points[i].z<1.0)//y<3 or laser_cloud.points[i].y>-5)
        {
            laser_cloud.points.erase(laser_cloud.points.begin()+i);
        }
        else i++;
    }
    cout<<laser_cloud.points.size()<<endl;*/
    laser_cloud_ = laser_cloud;
}
void curb_to_laser::pointcloudsToLaser(PointCloud& cloud, sensor_msgs::LaserScan& output)
{
    //adapted from turtlebot's cloud_to_scan.cpp
    output.header = cloud.header;
    output.angle_min = -M_PI;
    output.angle_max = M_PI;
    output.angle_increment = M_PI/180.0/2.0;
    output.time_increment = 0.0;
    output.scan_time = 1.0/30.0;
    output.range_min = 0.1;
    output.range_max = 80.0;

    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
    output.ranges.assign(ranges_size, output.range_max + 1.0);

    for (PointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
    {
        const float &x = it->x;
        const float &y = it->y;
        const float &z = it->z;

        if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
        {
            ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;
        }

        double range_sq = y*y+x*x;
        /*if (range_sq < 0.) {
            ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
            continue;
        }*/

        double angle = -atan2(-y, x);
        if (angle < output.angle_min || angle > output.angle_max)
        {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
            continue;
        }
        int index = (angle - output.angle_min) / output.angle_increment;


        if (output.ranges[index] * output.ranges[index] > range_sq)
            output.ranges[index] = sqrt(range_sq);
    }
}

void curb_to_laser::addCurbPoints(sensor_msgs::PointCloudConstPtr pc)
{
    sensor_msgs::PointCloud pc_temp;
    try
    {
        tf_.transformPointCloud(target_frame_, *pc, pc_temp);
        curb_points_.header = pc_temp.header;
        curb_points_.points.insert(curb_points_.points.begin(), pc_temp.points.begin(), pc_temp.points.end());
        curb_points_.points.resize(max_curb_points_);
    }
    catch (tf::TransformException &ex)
    {
        ROS_DEBUG("Failure %s\n", ex.what()); //Print exception which was caught
    }
}
void curb_to_laser::leftCurbCallback(sensor_msgs::PointCloudConstPtr left_pc)
{
    addCurbPoints(left_pc);
}

void curb_to_laser::rightCurbCallback(sensor_msgs::PointCloudConstPtr right_pc)
{
    addCurbPoints(right_pc);
}

void curb_to_laser::curbPointsDistanceThreshold(sensor_msgs::PointCloud& curb_pts, double distance)
{
    for(size_t i=0; i<curb_pts.points.size();)
    {
        double x = curb_pts.points[i].x;
        double y = curb_pts.points[i].y;
        if(sqrt(x*x+y*y) > curb_dist_) curb_pts.points.erase(curb_pts.points.begin()+i);
        else i++;
    }
}

void curb_to_laser::publishCurb(const ros::TimerEvent& event)
{
    sensor_msgs::PointCloud2 curb_points2;
    curb_points_pub_.publish(curb_points_);
    sensor_msgs::PointCloud curb_point_baselink;
    try
    {

        tf_.transformPointCloud("base_link", curb_points_, curb_point_baselink);

        //thresholding the distance of curb points to adjust the weighting
        curbPointsDistanceThreshold(curb_point_baselink, 10);

        sensor_msgs::convertPointCloudToPointCloud2(curb_point_baselink, curb_points2);
        curb_points2_pub_.publish(curb_points2);
    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
        return;
    }
    PointCloud pcl_xyz;
    sensor_msgs::PointCloud curb_points_with_lasers;
    //cout<<laser_cloud_.header.frame_id<<" "<<curb_point_baselink.header.frame_id<<endl;

    if(laser_cloud_.header.frame_id!=curb_point_baselink.header.frame_id) return;

    //combine both observation
    curb_points_with_lasers.header = curb_point_baselink.header;
    curb_points_with_lasers.points.insert(curb_points_with_lasers.points.begin(), curb_point_baselink.points.begin(), curb_point_baselink.points.end());
    curb_points_with_lasers.points.insert(curb_points_with_lasers.points.begin(), laser_cloud_.points.begin(), laser_cloud_.points.end());
    sensor_msgs::convertPointCloudToPointCloud2(curb_points_with_lasers, curb_points2);
    pcl::fromROSMsg(curb_points2, pcl_xyz);
    sensor_msgs::LaserScan curb_laser;
    pointcloudsToLaser(pcl_xyz, curb_laser);
    curb_laser_pub_.publish(curb_laser);
}

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "curb_to_laser");
    curb_to_laser curb2laser;
    return 0;
}
