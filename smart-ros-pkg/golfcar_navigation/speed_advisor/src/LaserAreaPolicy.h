#ifndef EAPEDCROSSINGPOLICY_H_
#define EAPEDCROSSINGPOLICY_H_

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <laser_geometry/laser_geometry.h>

#include "IntersectionPolicy.h"
#include "SvgBoundary.h"

class LaserAreaPolicy: public IntersectionPolicy
{
public:
    LaserAreaPolicy(std::string path_id);
    bool is_clear_to_go();
    void update_dist(double d);

private:
    double dist_to_pedcross_;
    const double dist_to_pedcross_threshold_;
    std::vector<geometry_msgs::Point32> boundary_;
    bool ped_crossing_free_;

    std::string global_frame_;

    ros::Publisher boundary_pub_, obs_pts_pub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::TransformListener tf_;
    laser_geometry::LaserProjection projector_;
    tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pc2_filter_;

    void pt_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& pc2);
    void laser_callback(const sensor_msgs::LaserScanConstPtr& scan_in);
    void process(const sensor_msgs::PointCloud &);
    void pub_boundary();
    void pub_obstacle_pts(const std::vector<geometry_msgs::Point32> &);
};

#endif /* EAPEDCROSSINGPOLICY_H_ */
