#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <fmutil/fm_math.h>
#include <std_msgs/Int64.h>

#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <feature_detection/clusters.h>
#include <feature_detection/cluster.h>

#include <cv.h>
#include <highgui.h>

#include "read_svg.h"
#include "pnpoly.h"
using namespace std;

class ped_clustering
{
public:
  tf::TransformListener tf_;
  tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  string global_frame_, laser_frame_id_, svg_file_;
  vector<Point32> boundary_;
  ros::Publisher boundary_pub_;
  ros::Publisher cloud_pub_, centroid_pub_, clusters_pub_;
  laser_geometry::LaserProjection projector_;
  cv::Mat ped_img_;
  double dist_thres_;
  vector<vector<geometry_msgs::Point32> > boundaries_;
  ped_clustering(){
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    private_nh.param("global_frame", global_frame_, string("/golfcart/map"));
    private_nh.param("laser_frame", laser_frame_id_, string("/golfcart/front_bottom_lidar"));
    private_nh.param("svg_file", svg_file_, string("utown_exp_launch_files/utown_plaza_ped_boundary.svg"));
    private_nh.param("dist_thres", dist_thres_, 1.0);
    laser_sub_.subscribe(nh, "/golfcart/front_bottom_scan", 10);
    boundary_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("/golfcart/ped_boundary",1, true);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, global_frame_, 10);
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>("/golfcart/laser_cloud", 1);
    centroid_pub_ = nh.advertise<sensor_msgs::PointCloud>("/golfcart/ped_centroid", 1);
    clusters_pub_ = nh.advertise<feature_detection::clusters>("/golfcart/pedestrian_clusters",1);
    tf_filter_->registerCallback(boost::bind(&ped_clustering::laserCallback, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.05));
    
    geometry_msgs::PolygonStamped boundary_msg;
    boundary_msg.header.stamp = ros::Time::now();
    boundary_msg.header.frame_id = "/golfcart/map";
    boundary_msg.header.seq = 1;
    svg_boundary svg(svg_file_.c_str(), 0.1);
    
    boundaries_ = svg.getAllPath();
    for(size_t i=0; i<boundaries_.size(); i++){
      boundary_msg.polygon.points.insert(boundary_msg.polygon.points.end(),
					 boundaries_[i].begin(), boundaries_[i].end());
    }
    boundary_pub_.publish(boundary_msg);
    ped_img_ = cv::Mat::zeros(2832, 2000, CV_8UC3);
    ros::spin();
  }
  
  ~ped_clustering(){
    //cv::imwrite("ped_img.png", ped_img_);
  }
  
  feature_detection::cluster addNewCluster(vector<geometry_msgs::Point32> &current_cluster, 
		     vector<geometry_msgs::Point32> &centroids){
    feature_detection::cluster new_cluster;
    new_cluster.points = current_cluster;
     if(current_cluster.size()>1){
	geometry_msgs::Point32 centroid;
	int last_cluster_no = current_cluster.size()-1;
	centroid.x = (current_cluster[last_cluster_no].x + current_cluster[0].x)/2.;
	centroid.y = (current_cluster[last_cluster_no].y + current_cluster[0].y)/2.;
	centroid.z = centroids.size();
	centroids.push_back(centroid);
	
	new_cluster.width = 0.5;
	new_cluster.height = 0.1;
	new_cluster.depth = 0.5;
	new_cluster.centroid = centroid;
    //the height for centroid is used for visualization earlier. Set it to zero
    //for correct representation
    new_cluster.centroid.z = 0.25;
      }
      current_cluster.clear();
      return new_cluster;
  }
  void segmentation(sensor_msgs::PointCloud &pc_filtered){
    feature_detection::clusters clusters;
    clusters.header = pc_filtered.header;
    if(pc_filtered.points.size()<1) return;
    geometry_msgs::Point32 last_pt = pc_filtered.points[0];
    vector<geometry_msgs::Point32> centroids;
    vector<geometry_msgs::Point32> current_cluster;
    for(size_t i=1; i<pc_filtered.points.size(); i++){
      double cur_x = pc_filtered.points[i].x;
      double cur_y = pc_filtered.points[i].y;
      double cur_dist = fmutil::distance(cur_x, cur_y,
		       last_pt.x, last_pt.y);
      
     if(cur_dist > dist_thres_){
	feature_detection::cluster new_cluster = addNewCluster(current_cluster, centroids);
	if(new_cluster.points.size() > 1) clusters.clusters.push_back(new_cluster);
      }
      last_pt = pc_filtered.points[i];
      current_cluster.push_back(pc_filtered.points[i]);
    }
    feature_detection::cluster new_cluster = addNewCluster(current_cluster, centroids);
    if(new_cluster.points.size()>1) clusters.clusters.push_back(new_cluster);
    sensor_msgs::PointCloud centroids_pc;
    centroids_pc.header = pc_filtered.header;
    centroids_pc.points = centroids;
    centroid_pub_.publish(centroids_pc);
    clusters_pub_.publish(clusters);
  }
  void laserCallback(const sensor_msgs::LaserScanConstPtr& scan_in){
    //cout<<"laserCallback"<<endl;
    sensor_msgs::PointCloud pc, pc_filtered;
    try{projector_.transformLaserScanToPointCloud(global_frame_, *scan_in, pc, tf_);}
    catch (tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}
    pc_filtered.header = pc.header;
    pc_filtered.header.frame_id = laser_frame_id_;
    for(size_t i=0; i<pc.points.size(); i++){
      /*int x = pc.points[i].x/0.1;
      int y = 2832 - pc.points[i].y/0.1;
      if(x>0 && y>0 && x<ped_img_.cols &&y<ped_img_.rows){
	if(ped_img_.at<cv::Vec3b>(y,x)[0]<255) ped_img_.at<cv::Vec3b>(y,x)[0]+=2;
	if(ped_img_.at<cv::Vec3b>(y,x)[1]<255) ped_img_.at<cv::Vec3b>(y,x)[1]+=2;
	if(ped_img_.at<cv::Vec3b>(y,x)[2]<255) ped_img_.at<cv::Vec3b>(y,x)[2]+=2;
      }*/
      bool point_in_poly = true;
      for(size_t j=0; j<boundaries_.size(); j++){
	if(pointInPolygon<geometry_msgs::Point32>(pc.points[i], boundaries_[j])){
	  point_in_poly = false;
	  break;
	}
      }
      if(point_in_poly) {
	PointStamped global_pt, local_pt;
	global_pt.header = pc.header;
	global_pt.point.x = pc.points[i].x;
	global_pt.point.y = pc.points[i].y;
	global_pt.point.z = pc.points[i].z;
	try{tf_.transformPoint(laser_frame_id_, global_pt, local_pt);}
        catch(tf::TransformException& e){ROS_INFO_STREAM(e.what());continue;}
        geometry_msgs::Point32 local_pt32;
	local_pt32.x = local_pt.point.x; local_pt32.y = local_pt.point.y;
	local_pt32.z = 0;
    //for some reason it is twice the length
   // if(fabs(local_pt32.x) < 10 && fabs(local_pt32.y) < 3+1.0)
	    pc_filtered.points.push_back(local_pt32);
     }
    }
    cloud_pub_.publish(pc_filtered);
    
    segmentation(pc_filtered);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "ped_clustering_laser");
  ped_clustering ped;
  return 0;
}
