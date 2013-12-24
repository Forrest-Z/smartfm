#include <math.h>

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include "golfcar_purepursuit.h"
#include <pnc_msgs/move_status.h>
#include <pnc_msgs/poi.h>
#include <fmutil/fm_math.h>
#include <StationPath.h>
#include <pnc_msgs/poi.h>

using namespace std;

class TrajectoryPlannerStandalone
{
  bool forward_, stopped_, goalreached_;
  int waypointPassed_;
  double frequency_, delay_;
  ros::Publisher move_status_pub_, cmd_steer_pub_, global_plan_pub_;
  ros::Publisher slowZone_pub_, poi_pub_;
  ros::Subscriber origin_destination_pt_sub_, g_plan_repub_sub_;
  golfcar_purepursuit::PurePursuit* pp_;
  string global_frame_, robot_frame_;
  tf::TransformListener* tf_;
  nav_msgs::Path rrt_path_;
  StationPaths sp_;
public:
  TrajectoryPlannerStandalone(){
    forward_ = true;
    stopped_ = false;
    goalreached_ = false;
    waypointPassed_ = -1;
    tf_ = new tf::TransformListener();
    ros::NodeHandle nh, priv_nh("~");
    move_status_pub_ = nh.advertise<pnc_msgs::move_status>("move_status",1);
    cmd_steer_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    global_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
    slowZone_pub_ = nh.advertise<geometry_msgs::PoseArray>("slowZone", 1, true);
    poi_pub_ = nh.advertise<pnc_msgs::poi>("poi", 1, true);
    g_plan_repub_sub_ = nh.subscribe("global_plan_repub", 1, &TrajectoryPlannerStandalone::repubCallback, this);
    origin_destination_pt_sub_ = nh.subscribe("move_base_simple/goal", 1, &TrajectoryPlannerStandalone::originDestinationCallback, this);
    priv_nh.param("global_frame", global_frame_, string("/map")); 
    priv_nh.param("robot_frame", robot_frame_, string("/base_link"));
    priv_nh.param("frequency", frequency_, 100.0);
    priv_nh.param("max_pose_delay", delay_, 0.015);
    
    pp_ = new golfcar_purepursuit::PurePursuit(global_frame_.c_str());
    
    //without a variable to hold the timer, the timer won't start!!!
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/frequency_), &TrajectoryPlannerStandalone::controlLoop, this);
  
    ros::spin();
  }
  
private:
  void repubCallback(nav_msgs::PathConstPtr p){
    ROS_INFO("New path received from rrt with size of: %d", (int)p->poses.size());
    rrt_path_ = *p;
  }
  void add_pts(vector<int>& pts, vector<pnc_msgs::sig_pt>* poi, int type)
  {
    pnc_msgs::sig_pt sig_pt;
    for(unsigned int i=0; i<pts.size(); i++)
    {
	sig_pt.points = pts[i];
	sig_pt.type = type;
	poi->push_back(sig_pt);
    }
  }
  void originDestinationCallback(geometry_msgs::PoseStampedConstPtr origin_dest){
    ROS_INFO("In make plan");
    std::vector<geometry_msgs::PoseStamped> plan;
    if(rrt_path_.poses.size() == 0){
      //all in according to the pixels from global map
      ROS_INFO("Normal plan");
      int start_number = (int)origin_dest->pose.position.x;
      int end_number = (int)origin_dest->pose.position.y;
      StationPath station_path = sp_.getPath(sp_.knownStations()(start_number),
					    sp_.knownStations()(end_number));
      geometry_msgs::PoseArray slowZones;
      slowZones.header.frame_id = global_frame_;
      slowZones.header.stamp = ros::Time::now();

      for(unsigned int i=0;i<sp_.slowZones_.size();i++)
      {
	  geometry_msgs::Pose pose;
	  pose.orientation.w = 1.0;
	  pose.position.x = sp_.slowZones_[i].x_;
	  pose.position.y = sp_.slowZones_[i].y_;
	  pose.position.z = sp_.slowZones_[i].r_;
	  slowZones.poses.push_back(pose);
      }

      slowZone_pub_.publish(slowZones);

      //publish point of interest
      pnc_msgs::poi poi;
      add_pts(station_path.leftsig_pts_, &poi.sig_pts, 0);
      add_pts(station_path.rightsig_pts_, &poi.sig_pts, 1);
      add_pts(station_path.offsig_pts_, &poi.sig_pts, 2);

      //poi.rightsig_pts.insert(poi.rightsig_pts.begin(),station_path.rightsig_pts_.begin(),station_path.rightsig_pts_.end());
      //poi.offsig_pts.insert(poi.offsig_pts.begin(),station_path.offsig_pts_.begin(),station_path.offsig_pts_.end());// = station_path.leftsig_pts_;
      poi.int_pts.insert(poi.int_pts.begin(),station_path.ints_pts_.begin(),station_path.ints_pts_.end());// = station_path.leftsig_pts_;

      poi_pub_.publish(poi);

      vector<geometry_msgs::PoseStamped> final_targets;
      geometry_msgs::PoseStamped ps;

      nav_msgs::Path p;
      p.poses.resize(station_path.size());
      for(unsigned int i=0; i<station_path.size(); i++)
      {
	  p.poses[i].header.frame_id = global_frame_;
	  p.poses[i].header.stamp = ros::Time::now();
	  p.poses[i].pose.position.x = station_path[i].x_;
	  p.poses[i].pose.position.y = station_path[i].y_;
	  p.poses[i].pose.orientation.w = 1.0;
	  geometry_msgs::PoseStamped pl;
	  pl.header.frame_id = global_frame_;
	  pl.header.stamp = ros::Time::now();
	  pl.pose.position.x = station_path[i].x_;
	  pl.pose.position.y = station_path[i].y_;
	  pl.pose.orientation.w = 1.0;
	  plan.push_back(pl);
      }
      ROS_INFO("Plan with %d points sent.", (int)plan.size());
      p.header.stamp = ros::Time();
      p.header.frame_id = global_frame_;

      //p.poses.push_back(start);
      //p.poses.push_back(goal);
      global_plan_pub_.publish(p);
      //plan.push_back(targets);
    }
    //send the rrt path to the local planner
    else {
      for(size_t i=0; i<rrt_path_.poses.size(); i++){
	geometry_msgs::PoseStamped ps_temp;
	ps_temp.header.frame_id = global_frame_;
	ps_temp.header.stamp = ros::Time::now();
	ps_temp.pose.position.x = rrt_path_.poses[i].pose.position.x;
	ps_temp.pose.position.y = rrt_path_.poses[i].pose.position.y;
	ps_temp.pose.orientation.w = 1.0;
	plan.push_back(ps_temp);
      }
     ROS_INFO("Plan with rrt %d points sent.",(int)plan.size());
    }
    
    pp_-> path_.poses = plan;
    pp_-> initialized_ = false;
    pp_-> dist_to_final_point = 100;
    pp_-> path_n_ =0;
    pp_-> path_.poses = plan;
  }
  
  void controlLoop(const ros::TimerEvent& event){
    
    geometry_msgs::Twist cmd_vel;
    tf::Stamped<tf::Pose> robot_pose;
    pnc_msgs::move_status move_status;
    move_status.path_exist = false;
    double steer_angle;
    if(pp_->path_.poses.size()>0 && getRobotPose(robot_pose)){
      geometry_msgs::PoseStamped robot_pose_msg;
      tf::poseStampedTFToMsg(robot_pose, robot_pose_msg);
      pp_->vehicle_base_ = robot_pose_msg.pose;
      double dist_to_goal;
      //tbc...
      bool path_exist = pp_->steering_control(&steer_angle, &dist_to_goal);
      move_status.dist_to_goal = dist_to_goal;
      move_status.dist_to_ints = 123;
      move_status.sig_type = -1;
      if(path_exist){
	move_status.steer_angle = steer_angle;
	move_status.obstacles_dist = 123;
	move_status.path_exist = true;
	if( waypointPassed_ != pp_->path_n_ )
	{
	    ROS_INFO("Path %d/%d", pp_->path_n_, (int)pp_->path_.poses.size()-1);
	    waypointPassed_ = pp_->path_n_;
	}
      }
    }
    else
    {
        move_status.path_exist = false;
        if( pp_->path_n_ < (int) pp_->path_.poses.size()-1 )
        {
            //move_status.emergency = -1;
            move_status.steer_angle = steer_angle;
            ROS_WARN("Steering control at unknown state");
        }
        else
        {

            move_status.steer_angle = 0;
            if(!goalreached_)
            {
                ROS_INFO("No path found");
                goalreached_=true;
            }
        }


    }
    move_status_pub_.publish(move_status);
    cmd_vel.angular.z = move_status.steer_angle;
    //cmd_steer_pub_.publish(cmd_vel);
  }
  
    
  bool getRobotPose(tf::Stamped<tf::Pose> &robot_pose) {
    robot_pose.setIdentity();
    tf::Stamped<tf::Pose> i_pose;
    i_pose.setIdentity();
    i_pose.frame_id_ = robot_frame_;
    i_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    try {
        tf_->transformPose(global_frame_, i_pose, robot_pose);
    }
    catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s", ex.what());
        return false;
    }
    catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s", ex.what());
        return false;
    }
    catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s", ex.what());
        return false;
    }
    // check robot_pose timeout
    if (current_time.toSec() - robot_pose.stamp_.toSec() > delay_) {
        ROS_WARN("PurePursuit transform timeout. Current time: %.4f, pose(%s) stamp: %.4f, tolerance: %.4f",
                 current_time.toSec(), global_frame_.c_str(), robot_pose.stamp_.toSec(), delay_);
        return false;
    }
    return true;
 }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "purepursuit_controller");
  TrajectoryPlannerStandalone tps;
  return 0;
}