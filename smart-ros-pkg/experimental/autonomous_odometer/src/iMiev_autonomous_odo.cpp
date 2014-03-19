#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <pnc_msgs/speed_contribute.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <long_control/PID_Msg.h>
using namespace std;

bool speed_status_positive = false;
double speedstatus_lastreceived_ = 0.0;
double joy_steer_=0.0, steer_ang_=0.0;
double joysteer_lastreceived_=0.0, steerang_lastreceived_=0.0;
double speed_err = 999.9;
geometry_msgs::Point last_point_;
void speedStatusCallback(pnc_msgs::speed_contribute speed_status){
  speedstatus_lastreceived_=ros::Time::now().toSec();
  if(speed_status.element == 0 /*no response*/ || speed_status.element == 2 /*no path*/
    || speed_status.element == 7 /*emergency*/ || speed_status.element == 15 /*manual*/)
    speed_status_positive = false;
  else
    speed_status_positive = true;
}

void joysteerCallback(std_msgs::Float64 joy_steer){
  joysteer_lastreceived_=ros::Time::now().toSec();
  joy_steer_ = joy_steer.data;
}

void steerangCallback(std_msgs::Float64 steer_ang){
  steerang_lastreceived_=ros::Time::now().toSec();
  steer_ang_ = steer_ang.data;
}

void pidCallback(long_control::PID_Msg pid){
  speed_err = pid.vel_err;
}

double autonomous_dist_ = 0.0;
double last_steer_ang_ = 0.0;
void amclposeCallback(geometry_msgs::PoseWithCovarianceStamped amcl_pose){
  geometry_msgs::Point cur_point = amcl_pose.pose.pose.position;
  double x = last_point_.x - cur_point.x;
  double y = last_point_.y - cur_point.y;
  double dist = sqrt(x*x + y*y);
  bool dist_flag = dist < 1.0;
  bool steer_flag = fabs(joy_steer_ - steer_ang_) < 5.0 || fabs(last_steer_ang_ - steer_ang_)>1e-3;
  bool speed_flag = fabs(speed_err) < 1.0;
  if(dist_flag && steer_flag && speed_flag && speed_status_positive)
    autonomous_dist_+=dist;
  cout<<"Distance_travel "<<dist<<" joysteer "<<joy_steer_<<" steerang "<<steer_ang_<<" speed_stat "<<speed_status_positive
  <<" speed_err "<<speed_err<<" auto_dist "<<autonomous_dist_<<endl;
  //add time spent in autonomous mode, maximum and average speed
  last_steer_ang_=steer_ang_;
  last_point_ =cur_point ;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "iMiev_autonomous_odo");

  ros::NodeHandle nh;
  ros::Subscriber joysteer_sub = nh.subscribe("iMiev/pronto/joy_steer", 10, joysteerCallback);
  ros::Subscriber  steerang_sub = nh.subscribe("iMiev/pronto/steerang", 10, steerangCallback);
  ros::Subscriber amclpose_sub = nh.subscribe("iMiev/amcl_pose", 10, amclposeCallback);
  ros::Subscriber speed_status = nh.subscribe("iMiev/speed_status", 10, speedStatusCallback);
  ros::Subscriber pid_sub = nh.subscribe("iMiev/pid_term", 10, pidCallback);
  ros::spin();

  return 0;
}