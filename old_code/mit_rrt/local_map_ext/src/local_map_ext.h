/*
 * local_map_ext.h
 *
 *  Created on: Aug 15, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <cv.h>
#include <highgui.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Bool.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>

#include <fmutil/fm_math.h>

using namespace cv;
using namespace std;

class LocalMap
{
    public:
        LocalMap(double height, double width, double res);
    private:
        nav_msgs::OccupancyGrid local_map_;
        nav_msgs::OccupancyGrid prior_map_;
        void pointcloudCallback(sensor_msgs::PointCloud2ConstPtr pc);
        tf::MessageFilter<sensor_msgs::PointCloud2> *pointcloud_filter_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
        ros::Subscriber norminal_lane_sub_;
        void laserCallback(sensor_msgs::LaserScanConstPtr pc);
        tf::MessageFilter<sensor_msgs::LaserScan> *laser_filter_;
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
        laser_geometry::LaserProjection projector_;
        
        int updateMapSkipMax;
        int updateMapSkip;

        void updateMap(sensor_msgs::PointCloud& pc);
        void addPointToMap(geometry_msgs::Point32 map_p);
        void addPointToMap(geometry_msgs::Point32 map_p, int occ);
        bool getRobotPose(tf::Stamped<tf::Pose> &odom_pose) const;

        tf::TransformListener tf_;
        string global_frame_, local_frame_, base_frame_;
        ros::Publisher map_pub_, prior_pts_pub_, map_pts_pub_;
        ros::NodeHandle nh_;

        sensor_msgs::PointCloud prior_pts_, local_map_pts_;
        vector<int> prior_obs_master_, prior_obs_;

        tf::StampedTransform transform_;

        //Future dating to allow slower sending w/o timeout
        ros::Duration *tf_sleeper_;

        tf::TransformBroadcaster broadcaster_;

        void timerCallback(const ros::TimerEvent &event);
        void publishLocalMapPts();
        void norminalLane(std_msgs::Bool norminal_lane);
        double height_, width_;
        void updatePriorObsWithLane(bool norminal);
};
