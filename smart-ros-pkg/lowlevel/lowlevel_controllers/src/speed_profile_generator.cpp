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
ros::Publisher speed_pub;


void targetSpeedCallback(std_msgs::Float32 target_speed){
  target_speed_ = target_speed.data;
}
void callback(const ros::TimerEvent & event)
{
    double v2, j;
    double dt = (event.current_real - event.last_real).toSec();
    //determine intermediate speed target before changing state. 
    //Ref: http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=1026947&url=http%3A%2F%2Fieeexplore.ieee.org%2Fiel5%2F7974%2F22065%2F01026947.pdf%3Farnumber%3D1026947
    int state=0;
    if(target_speed_>speed_now_){
      state=1;
      v2 = target_speed_ - (max_acc_*max_acc_)/(2*max_jerk_);
      if(speed_now_ >= v2) j = -max_jerk_;
      else j = max_jerk_;
    }
    else if(target_speed_<speed_now_){
      state=2;
      v2 = target_speed_ + (max_acc_*max_acc_)/(2*max_jerk_);
      if(speed_now_ <= v2) j = max_jerk_;
      else j = -max_jerk_;
    }
    else {
      j = 0;
      acc_now_ = 0;
    }
    acc_now_ += j * dt;
    if(acc_now_ >= max_acc_){
      j = 0;
      acc_now_ = max_acc_;
    }
    else if(acc_now_ <= -max_acc_){
      j = 0;
      acc_now_ = -max_acc_;
    }
    speed_now_ += acc_now_ * dt + 0.5 * j * dt * dt;
    if(fabs(speed_now_-target_speed_)<0.01) speed_now_ = target_speed_;
    //std::cout<<v2<<" "<<state<<" "<<acc_now_<<" "<<dt<<" "<<j<<" "<<speed_now_<<std::endl;
    geometry_msgs::Twist tw;
    tw.linear.x = speed_now_;
    speed_pub.publish(tw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_profile");
    target_speed_ = 0;
    speed_now_ = acc_now_ = 0;
    double frequency;
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
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
