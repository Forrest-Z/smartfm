#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

using namespace std;

class PathRepub{
public:
  PathRepub() : tf_(), target_frame_("iMiev/map"){
    
    path_repub_sub_ = nh_.subscribe("global_plan_repub", 1, &PathRepub::pathCallback, this);
    path_repub_ = nh_.advertise<nav_msgs::Path>("global_plan_repub_h_zero", 1);
    local_map_pts_repub_ = nh_.advertise<sensor_msgs::PointCloud>("local_map_pts_repub", 1);
    pc_sub_.subscribe(nh_, "local_map_pts", 10);
    pc_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(pc_sub_, tf_, target_frame_, 10);
    pc_filter_->registerCallback(boost::bind(&PathRepub::pcCallback, this, _1));
  }
  
private:
  ros::Publisher path_repub_, local_map_pts_repub_;  
  ros::Subscriber path_repub_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud> pc_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<sensor_msgs::PointCloud>* pc_filter_;
  string target_frame_;
  ros::NodeHandle nh_;
  void pathCallback(nav_msgs::Path path){
    for(size_t i=0; i<path.poses.size(); i++)
      path.poses[i].pose.position.z = 0.0;
    path_repub_.publish(path);
  }

  void pcCallback(const sensor_msgs::PointCloudConstPtr& pc_callback){
    sensor_msgs::PointCloud pc = *pc_callback;
    sensor_msgs::ChannelFloat32 ch;
    ch.name = "intensities";
    for(size_t i=0; i<pc.points.size(); i++){
      ch.values.push_back(pc.points[i].z);
      pc.points[i].z = 0;
    }
    pc.channels.push_back(ch);
    local_map_pts_repub_.publish(pc);
  }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "path_repub");
  PathRepub path_repub;
  ros::spin();
  return 0;
}
