#include "LaserAreaPolicy.h"

#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PolygonStamped.h>

using namespace std;

LaserAreaPolicy::LaserAreaPolicy(string path_id)
: dist_to_pedcross_(1e6), dist_to_pedcross_threshold_(10), ped_crossing_free_(false)
{
    ros::NodeHandle private_nh("~"), nh;

    private_nh.param("global_frame", global_frame_, string("map"));
    cloud_sub_.subscribe(nh, "laser_area_policy/cloud", 10);
    tf_pc2_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_sub_, tf_, global_frame_, 10);
    tf_pc2_filter_->registerCallback(boost::bind(&LaserAreaPolicy::pt_cloud_callback, this, _1));
    tf_pc2_filter_->setTolerance(ros::Duration(0.05));

    boundary_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("ped_boundary",1, true);
    obs_pts_pub_ = nh.advertise<sensor_msgs::PointCloud>("ped_obstacles", 1, true);

    laser_sub_.subscribe(nh, "laser_area_policy/scan", 10);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, global_frame_, 10);
    tf_filter_->registerCallback(boost::bind(&LaserAreaPolicy::laser_callback, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.05));

    string svg_file;
    private_nh.param("svg_file", svg_file, string("~/SvgPath/path_def.svg"));
    SvgBoundary svg_boundary(svg_file, 0.1);
    boundary_ = svg_boundary.getPath(path_id);
}

bool LaserAreaPolicy::is_clear_to_go()
{
    ROS_DEBUG_NAMED("laser_area", "dist_to_pedcross=%f, thr=%f, ped_crossing_free=%d",
            dist_to_pedcross_, dist_to_pedcross_threshold_, ped_crossing_free_);
    return dist_to_pedcross_<dist_to_pedcross_threshold_ && ped_crossing_free_;
}

void LaserAreaPolicy::update_dist(double d)
{
    dist_to_pedcross_ = d;
}

void LaserAreaPolicy::pub_boundary()
{
    geometry_msgs::PolygonStamped boundary_msg;
    boundary_msg.header.stamp = ros::Time::now();
    boundary_msg.header.frame_id = global_frame_;
    boundary_msg.polygon.points = boundary_;
    boundary_pub_.publish(boundary_msg);
}

void LaserAreaPolicy::pub_obstacle_pts(const vector<geometry_msgs::Point32> & pts)
{
    sensor_msgs::PointCloud msg;
    msg.header.frame_id = global_frame_;
    msg.header.stamp = ros::Time::now();
    msg.points = pts;
    obs_pts_pub_.publish(msg);
}

bool pointInPolygon(geometry_msgs::Point p, const vector<geometry_msgs::Point32> & poly)
{
    bool oddNodes = false;

    for (unsigned i=0, j=poly.size()-1; i<poly.size(); i++)
    {
        if (    (
                    (   poly[i].y< p.y && poly[j].y>=p.y    )
                    ||
                    (   poly[j].y< p.y && poly[i].y>=p.y    )
                )
                &&
                (   poly[i].x<=p.x || poly[j].x<=p.x    )
            )
        {
            double m = (poly[j].y-poly[i].y)/(poly[j].x-poly[i].x);
            oddNodes ^= (poly[i].x + (p.y-poly[i].y)/m < p.x);
        }
        j=i;
    }

    return oddNodes;
}

void LaserAreaPolicy::process(const sensor_msgs::PointCloud & pc)
{
    std::vector<geometry_msgs::Point32> obs_pts;
    std::vector<geometry_msgs::Point32>::const_iterator it = pc.points.begin();
    for( ; it != pc.points.end(); ++it )
    {
        geometry_msgs::PointStamped global_pt, local_pt;
        local_pt.header = pc.header;

        local_pt.point.x = it->x;
        local_pt.point.y = it->y;
        local_pt.point.z = it->z;
        try
        {
            tf_.transformPoint(global_frame_, local_pt, global_pt);
        }
        catch(tf::TransformException& e)
        {
            ROS_INFO_STREAM_NAMED("laser_area", e.what());
            continue;
        }

        //log shows the wn_PnPoly falls into the boundary only if output is 1
        if( pointInPolygon(global_pt.point, boundary_) )
            obs_pts.push_back(*it);
    }

    ROS_DEBUG_NAMED("laser_area", "%d points in ped_boundary", obs_pts.size());
    pub_boundary();
    pub_obstacle_pts(obs_pts);
    ped_crossing_free_ = obs_pts.empty();
}

void LaserAreaPolicy::pt_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    ROS_DEBUG_NAMED("laser_area", "pt_cloud_callback");
    if(dist_to_pedcross_ > dist_to_pedcross_threshold_)
        return;
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(*pc2, pc);
    process(pc);
}

void LaserAreaPolicy::laser_callback(const sensor_msgs::LaserScanConstPtr& scan_in)
{
    ROS_DEBUG_NAMED("laser_area", "laser_callback");
    if(dist_to_pedcross_ > dist_to_pedcross_threshold_)
        return;

    sensor_msgs::PointCloud pc;
    try
    {
        projector_.transformLaserScanToPointCloud(global_frame_, *scan_in, pc, tf_);
    }
    catch (tf::TransformException& e)
    {
        ROS_INFO_STREAM_NAMED("laser_area", e.what());
        return;
    }
    process(pc);
}
