/* NUS_Golf_Cart 2011-12-15
 */

#ifndef __SCAN_LDMRS__
#define __SCAN_LDMRS__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace road_detection{

class scan_ldmrs{

public:
	scan_ldmrs();
	~scan_ldmrs();

private:
    ros::NodeHandle             nh_, private_nh_;

    bool                        scan1_income_;
    bool                        scan2_income_;
    double                      assemble_time_thresh_;
    std::string                 ldmrs_single_id_;

    ros::Subscriber             scan_sub1_;
    ros::Subscriber             scan_sub2_;
    sensor_msgs::LaserScan		scan1_;
    sensor_msgs::LaserScan		scan2_;
    sensor_msgs::LaserScan		assembled_scan_;
    ros::Publisher              scan_pub_;
    void ldmrs_callBack1(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    void ldmrs_callBack2(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    void ldmrs_assemble();
    
};
};

#endif 
