/** Speed profile generation.
 *
 * Generates a smooth speed profile: accelerate, maintain, decelerate
 * based on input with given contraint of max_jerk, max_acc, max_vel
 * The given input is target velocity
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
double max_jerk_, max_acc_, max_speed_;
double target_speed_;
double speed_now_, acc_now_;
double v0_, v1_, v2_;
double intermediate_speed_;
double start_time_;
ros::Publisher speed_pub;
int speed_interval_;
int acceleration_sign_;
bool target_active_;
bool fast_target_;
using namespace std;
int counter_;
void targetSpeedCallback(std_msgs::Float32 target_speed){
  double new_target = target_speed.data;
  target_active_ = true;
  
  if(target_speed_ == new_target) target_active_ = false;
  target_speed_ = new_target;
  
  
  if(target_active_){
    if(speed_interval_ == 4) speed_interval_ = 1;
    if(new_target > speed_now_) acceleration_sign_ = 1;
    else if(new_target < speed_now_) acceleration_sign_ = -1;
    double j = acceleration_sign_ * max_jerk_;
    double a = acceleration_sign_ * max_acc_;
    double a_max_sq = max_acc_ * max_acc_;
    ROS_INFO_STREAM("Target changed "<<speed_interval_<<": amount="<<target_speed_-v0_<<" j="<<j<<" speed_now="<<speed_now_<<" acc_now="<<acc_now_);
    switch(speed_interval_){
      case 1:
	v0_ = speed_now_ - acc_now_*acc_now_/(2*j);
	speed_interval_ = 1;
	break;
      case 2:
	v0_ = speed_now_ - a_max_sq/(2*j);
	speed_interval_ = 2;
	break;
      case 3:
	v0_ = speed_now_ - acc_now_*acc_now_/(2*j);
	speed_interval_ = 1;
	break;
    }
    if( fabs(target_speed_-v0_)>= a_max_sq/max_jerk_){
      fast_target_ = false;
      v1_ = v0_+ a_max_sq/(2*j);
      v2_ = target_speed_ -(a_max_sq/(2*j));
      ROS_INFO_STREAM("Normal target v0: "<<v0_<<" vt: "<<target_speed_<<" v1: "<<v1_<<" v2: "<<v2_); 
    }
    else {
      fast_target_ = true;
      v1_ = (target_speed_ +v0_)/2;
      ROS_INFO_STREAM("Quick target v0: "<<v0_<<" vt: "<<target_speed_<<" v1: "<<v1_);
    }
    v0_ = target_speed_;
    
    target_active_ = false;
  }
}

double newSpeedDynamicModel(double j, double dt){
  acc_now_ += j * dt;
  double new_speed = speed_now_ + acc_now_ * dt + 0.5 * j * dt * dt;
  return new_speed;
}

void callback(const ros::TimerEvent & event)
{
    double dt = (event.current_real - event.last_real).toSec();
    double j = acceleration_sign_ * max_jerk_;
    double a = acceleration_sign_ * max_acc_;
    double a_max_sq = max_acc_ * max_acc_;
    bool change_state = false;
    //Ref: http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=1026947&url=http%3A%2F%2Fieeexplore.ieee.org%2Fiel5%2F7974%2F22065%2F01026947.pdf%3Farnumber%3D1026947
    
    
    
    switch (speed_interval_){
      case 1:
	j = j;
	if(acceleration_sign_>0){
	  if(speed_now_>v1_) change_state = true;
	}
	else{
	  if(speed_now_<v1_) change_state = true;
	}
	if(change_state){
	  
	  if(fast_target_){
	    j = -j;
	    speed_interval_ = 3;
	  }
	  else {
	    j = 0;
	    speed_interval_ = 2;
	  }
	  ROS_INFO_STREAM("Change state from 1 to "<<speed_interval_);
	}
	break;
      case 2:
	j = 0;
	if(acceleration_sign_>0){
	  if(speed_now_>v2_) change_state = true;
	}
	else{
	  if(speed_now_<v2_) change_state = true;
	}
	if(change_state){
	  j = -j;
	  acc_now_ = a;
	  speed_interval_ = 3;
	  ROS_INFO_STREAM("Change state from 2 to 3");
	}
	break;
      case 3:
	j = -j;
	if(acceleration_sign_>0){
	  if(acc_now_<=0 || speed_now_>target_speed_) change_state = true;
	}
	else{
	  if(acc_now_>=0 || speed_now_<target_speed_) change_state = true;
	}
	if(change_state){
	  speed_interval_ = 4;
	  acc_now_ = 0;
	  j = 0;
	  speed_now_ = target_speed_;
	  ROS_INFO_STREAM("Change state from 3 to 4");
	}
	break;
      case 4:
	speed_now_ = target_speed_;
	j = 0;
	acc_now_ = 0;
	break;
    }
    
    speed_now_ = newSpeedDynamicModel(j, dt);
    //cout<<"State: "<<speed_interval_<<", v1: "<<v1_<<", v2: "<<v2_<<" acc_now: "<<acc_now_<<" speed_now: "<<speed_now_<<endl;
    if(counter_++ ==10){
      geometry_msgs::Twist tw;
      tw.linear.x = speed_now_;
      tw.linear.y = acc_now_;
      tw.linear.z = j;
      speed_pub.publish(tw);
      counter_ = 1;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_profile");
    target_speed_ = 0;
    speed_interval_ = 4;
    speed_now_ = acc_now_ = 0;
    acceleration_sign_ = 1;
    target_active_ = false;
    fast_target_ = false;
    double frequency;
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    speed_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    private_nh.param("max_jerk", max_jerk_, 0.5);
    private_nh.param("max_acc", max_acc_, 0.5);
    private_nh.param("max_speed", max_speed_, 1.5);
    private_nh.param("frequency", frequency, 1000.0);
    ros::Timer timer = n.createTimer(ros::Duration(1/frequency), callback);
    ros::Subscriber target_sub = n.subscribe("target_speed", 1, targetSpeedCallback);
    
    ros::spin();
    geometry_msgs::Twist tw;
    tw.linear.x = 0;
    speed_pub.publish(tw);
    return 0;
}
