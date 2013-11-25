#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <feature_detection/clusters.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include "read_svg.h"
#include "pnpoly.h"
class SimpleObjectDetection
{
public:
  
  SimpleObjectDetection(){
    ros::NodeHandle private_nh("~");
    private_nh.param("global_frame", global_frame_, string("/map"));
    private_nh.param("laser_frame", laser_frame_id_, string("/scan"));
    string svg_file;
    private_nh.param("svg_file", svg_file, string(""));

    laser_sub_.subscribe(nh_, "input_scan", 10);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, global_frame_, 10);
    tf_filter_->registerCallback(boost::bind(&SimpleObjectDetection::laserCallback, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.05));
    
    boundary_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("detection_boundary",1, true);
    poi_pub_= nh_.advertise<sensor_msgs::PointCloud>("pedestrian_poi",1);
    
    geometry_msgs::PolygonStamped boundary_msg;
    boundary_msg.header.stamp = ros::Time::now();
    boundary_msg.header.frame_id = "/map";
    boundary_msg.header.seq = 1;

    
    svg_boundary svg(svg_file.c_str(), 0.1);
    boundary_ = svg.getPath("crossing_boundary");
    boundary_msg.polygon.points = boundary_;
    

    boundary_pub_.publish(boundary_msg);
    ros::spin();
  }
  
private:
  ros::NodeHandle nh_;
  tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  ros::Publisher clusters_pub_, boundary_pub_, poi_pub_;
  tf::TransformListener tf_;
  sensor_msgs::PointCloud laser_global_pc_;
  string global_frame_, laser_frame_id_;
  vector<Point32> boundary_;
  laser_geometry::LaserProjection projector_;
  void laserCallback(const sensor_msgs::LaserScanConstPtr& scan_in){
    sensor_msgs::PointCloud pc;

    try{projector_.transformLaserScanToPointCloud(global_frame_, *scan_in, pc, tf_);}
    catch (tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}

    //interleave based on map frame
    //based on initial impression, it is quite good. Points appears to be more stable

    if(scan_in->header.seq%2)
    {
    	
        laser_global_pc_.points.clear();
        laser_global_pc_ = pc;
        return;
    }
    else
    {
    	laser_global_pc_.header.stamp = scan_in->header.stamp;
    	laser_global_pc_.points.insert(laser_global_pc_.points.end(), pc.points.begin(), pc.points.end());
    }
    
    //cout<<"laser_scan seq "<<scan_in->header.seq<<endl;
    sensor_msgs::PointCloud filtered_pts;
    for(size_t i=0; i<pc.points.size(); i++){
      if(!pointInPolygon<Point32>(pc.points[i], boundary_))
            		continue;
      filtered_pts.points.push_back(pc.points[i]);
    }
    filtered_pts.header = pc.header;
    poi_pub_.publish(filtered_pts);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SimpleObjectDetection");
  SimpleObjectDetection sod;
  return 0;
}