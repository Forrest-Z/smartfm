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
double v0_;
double change_interval_time_;
double start_time_;
ros::Publisher speed_pub;
int speed_interval_;
bool fast_target_;

using namespace std;

void checkForNewTarget(double new_target){
  fast_target_ = (fabs(new_target - speed_now_) < (max_acc_*max_acc_/max_jerk_));
  double time_now = ros::Time::now().toSec();
  if(fast_target_) change_interval_time_ = time_now + sqrt(fabs(max_speed_-v0_)/max_jerk_);
  else change_interval_time_ = time_now + max_acc_/max_jerk_;
  
  if(new_target>target_speed_) speed_interval_ = 1;
  else if(new_target < target_speed_) speed_interval_ = 5;
  cout<<"New target: "<<speed_interval_<<" "<<change_interval_time_-start_time_<<endl;
    
}

void targetSpeedCallback(std_msgs::Float32 target_speed){
  checkForNewTarget(target_speed.data);
  target_speed_ = target_speed.data;
}

double newSpeedDynamicModel(double j, double dt){
  acc_now_ += j * dt;
  double new_speed = speed_now_ + acc_now_ * dt + 0.5 * j * dt * dt;
  return new_speed;
}

void callback(const ros::TimerEvent & event)
{
    double dt = (event.current_real - event.last_real).toSec();
    double time_now = ros::Time::now().toSec();
    bool time_up = ((time_now - change_interval_time_) > 0);
    double j;
    
    //Ref: http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=1026947&url=http%3A%2F%2Fieeexplore.ieee.org%2Fiel5%2F7974%2F22065%2F01026947.pdf%3Farnumber%3D1026947
    switch (speed_interval_){
      case 1:
	j = max_jerk_;
	if(time_up) {
	  if(fast_target_) {
	    j = -max_jerk_;
	    speed_interval_=3;
	    change_interval_time_ +=sqrt((max_speed_-v0_)/max_jerk_);
	  }
	  else {
	    j = 0;
	    speed_interval_++;
	    change_interval_time_ += (target_speed_ - 2 * v0_)/max_acc_ - max_acc_/max_jerk_;
	  }
	}
	break;
      case 2:
	j = 0;
	if(time_up){
	  j = -max_jerk_;
	  speed_interval_++;
	  change_interval_time_ +=  max_acc_/max_jerk_;
	}
	break;
      case 3:
	j = -max_jerk_;
	if(time_up) {
	  j = 0;
	  speed_interval_++;
	}
	break;
      case 4:
	v0_ = target_speed_;
	speed_now_ = target_speed_;
	break;
      case 5:
	j = -max_jerk_;
	if(time_up){
	  if(fast_target_){
	    j = -max_jerk_;
	    speed_interval_=7;
	    change_interval_time_ += sqrt((v0_-max_speed_)/max_jerk_);
	  }
	  else {
	    j = 0;
	    speed_interval_++;
	    change_interval_time_ += ( 2 * v0_ - target_speed_)/max_acc_ - max_acc_/max_jerk_;
	  }
	}
	break;
      case 6:
	j = 0;
	if(time_up){
	  j = -max_jerk_;
	  speed_interval_++;
	  change_interval_time_ += max_acc_/max_jerk_;
	}
	break;
      case 7:
	j = -max_jerk_;
	if(time_up) {
	  j = 0;
	  speed_interval_ = 4;
	}
	break;
    }
    
    if(speed_interval_!=4)
      speed_now_ = newSpeedDynamicModel(j, dt);
    cout<<speed_interval_<<" "<<j<<" "<<change_interval_time_-start_time_<<" "<<time_now-start_time_<<" "<<acc_now_<<endl;
    geometry_msgs::Twist tw;
    tw.linear.x = speed_now_;
    speed_pub.publish(tw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_profile");
    target_speed_ = 0;
    speed_interval_ = 4;
    speed_now_ = acc_now_ = 0;
    fast_target_ = 0;
    double frequency;
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    change_interval_time_ = start_time_ = ros::Time::now().toSec();
    speed_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    private_nh.param("max_jerk", max_jerk_, 1.0);
    private_nh.param("max_acc", max_acc_, 0.5);
    private_nh.param("max_speed", max_speed_, 1.5);
    private_nh.param("frequency", frequency, 100.0);
    ros::Timer timer = n.createTimer(ros::Duration(1/frequency), callback);
    ros::Subscriber target_sub = n.subscribe("target_speed", 1, targetSpeedCallback);
    
    ros::spin();
    geometry_msgs::Twist tw;
    tw.linear.x = 0;
    speed_pub.publish(tw);
    return 0;
}
