/* NUS_Golf_Cart 2011-11-30
 * Description: "curb_detect" is 2nd version of original "road_detect";
 * adopting a more robust, neat, and systematic way to detect curb features, road_surface (by-product), and so on.
 */

#ifndef __SCAN_ASSEMBLE__
#define __SCAN_ASSEMBLE__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
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

class scan_assemble{

public:
	scan_assemble();
	~scan_assemble();

private:
   ros::Subscriber              scan_sub_;
   ros::Publisher               scan_pub_;
   sensor_msgs::LaserScan		laser_scan1_;
   sensor_msgs::LaserScan		laser_scan2_;
   sensor_msgs::LaserScan		assembled_scan_; 
   double                       assemble_time_thresh_;
   int                          laser_income_;
   void assemble_scans(const sensor_msgs::LaserScan::ConstPtr& scan_in);
};
};

#endif 
