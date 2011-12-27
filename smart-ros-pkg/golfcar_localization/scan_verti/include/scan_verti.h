/* NUS_Golf_Cart 2011-12-11
 */

#ifndef __SCAN_VERTI__
#define __SCAN_VERTI__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"


namespace road_detection{

class scan_verti{

public:
	scan_verti();
	~scan_verti();

private:
    ros::NodeHandle nh_, private_nh_;

    std::string                 base_frame_id_;
    std::string                 verti_base_frame_id_;
    std::string                 laser_frame_id_;
    std::string                 verti_laser_frame_id_;
    double                      LowHeight_Thresh_;
    double                      HighHeight_Thresh_;
    int                         UseNum_Thresh_;

    ros::Subscriber             imu_subscriber_;
    sensor_msgs::Imu            latest_imu_msg_;

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::TransformListener tf_; 
    tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    ros::Time                   laser_time_;
    sensor_msgs::LaserScan		raw_scan_;
    sensor_msgs::PointCloud     raw_laser_pcl_;
    sensor_msgs::LaserScan		verti_scan_;
    sensor_msgs::PointCloud     verti_pcl_;
  
    laser_geometry::LaserProjection projector_; 
    ros::Publisher              verti_scan_pub_;     
    ros::Publisher              vertiPcl_pub_;

    void imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg);
    void tfGeneration();
    void consVertiScan();
    void addPttoScan(geometry_msgs::PointStamped &verti_pt, sensor_msgs::LaserScan &verti_scan);
    tf::Transformer transformer_;
    tf::TransformBroadcaster tf_broadcaster_;
};
};

#endif 
