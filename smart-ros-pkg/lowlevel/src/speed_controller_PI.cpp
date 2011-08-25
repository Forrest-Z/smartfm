#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <lse_xsens_mti/imu_rpy.h>

#define BOUND(x,m,M) ( (x)>(M) ? (M) : (x)<(m) ? (m) : (x) )


class PID_Speed
{
  public:
    PID_Speed();

  private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber odovel_sub_;
    ros::Subscriber emergency_sub_;
    ros::Subscriber rpy_sub_;
    ros::Publisher throttle_pub_;
    ros::Publisher brakepedal_pub_;

    double kp_, ki_, ki_sat_, coeff_th_, coeff_bp_;
    double throttle_zero_thres_, brake_zero_thres_, full_brake_thres_;
    double tau_v_;
    double pitch_param1_, pitch_param2_;

    ros::Time time_pre_;
    double e_pre_;
    double ei_;
    double v_filtered_;

    double cmd_vel_;
    double pitch_last_;
    bool emergency_;

    void cmdVelCB(geometry_msgs::Twist);
    void rpyCB(lse_xsens_mti::imu_rpy);
    void odovelCB(std_msgs::Float64);
    void emergencyCB(std_msgs::Bool);
};

PID_Speed::PID_Speed()
{
  cmd_vel_sub_= nh_.subscribe("cmd_vel", 1, &PID_Speed::cmdVelCB, this);
  odovel_sub_= nh_.subscribe("odovel", 1, &PID_Speed::odovelCB, this);
  emergency_sub_ = nh_.subscribe("emergency", 1, &PID_Speed::emergencyCB, this);
  rpy_sub_ = nh_.subscribe("imu/rpy", 1, &PID_Speed::rpyCB, this);
  throttle_pub_ = nh_.advertise<std_msgs::Float64>("throttle_volt", 1);
  brakepedal_pub_ = nh_.advertise<std_msgs::Float64>("brake_angle", 1);

  ros::NodeHandle n("~");
  if(!n.getParam("kp",kp_)) kp_ = 0.15;
  if(!n.getParam("ki",ki_)) ki_ = 0.2;
  if(!n.getParam("ki_sat",ki_sat_)) ki_sat_ = 0.7;
  if(!n.getParam("coeff_throttle",coeff_th_)) coeff_th_ = 3.3;
  if(!n.getParam("coeff_brakepedal",coeff_bp_)) coeff_bp_ = 120;
  if(!n.getParam("throttleZeroThres", throttle_zero_thres_)) throttle_zero_thres_ = 0.3;
  if(!n.getParam("brakeZeroThres", brake_zero_thres_)) brake_zero_thres_ = 5.0;
  if(!n.getParam("fullBrakeThres", full_brake_thres_)) full_brake_thres_ = 0.25;
  if(!n.getParam("tau_v", tau_v_)) tau_v_ = 0.2;
  if(!n.getParam("pitch_param1", pitch_param1_)) pitch_param1_ = 0;
  if(!n.getParam("pitch_param2", pitch_param2_)) pitch_param2_ = -4;

  std::cout<<"kp: "<<kp_<<" ki: "<<ki_<<" ki_sat: "<<ki_sat_<<"\n";
  std::cout<<"coeff_th: "<<coeff_th_<<" coeff_bp: "<<coeff_bp_<<"\n";
  std::cout<<"throttle_threshold: "<<throttle_zero_thres_<<" brake_threshold: "<<brake_zero_thres_<<"\n";
  std::cout<<"tau_v: "<<tau_v_<<"\n";

  time_pre_ = ros::Time::now();
  e_pre_ = 0;
  ei_ = 0;
  v_filtered_ = 0;

  cmd_vel_ = 0;
  pitch_last_ = 0;
  emergency_ = false;
}

void PID_Speed::emergencyCB(std_msgs::Bool m)
{
  emergency_ = m.data;
}

void PID_Speed::cmdVelCB(geometry_msgs::Twist cmd_vel)
{
  cmd_vel_ = cmd_vel.linear.x;
}

void PID_Speed::rpyCB(lse_xsens_mti::imu_rpy rpy)
{
  pitch_last_ = rpy.pitch;
}

void PID_Speed::odovelCB(std_msgs::Float64 m)
{
  float odovel = m.data;
  std_msgs::Float64 throttle;
  std_msgs::Float64 brake;

  if(emergency_ || (cmd_vel_ <= 0 && odovel <= full_brake_thres_))
  {
    throttle.data = 0;
    brake.data = -coeff_bp_;
    cmd_vel_ = 0;
    time_pre_ = ros::Time::now();
    e_pre_ = 0;

    // when starting from a stopped position, initialization is important
    // especially, uphill is tricky
    if(pitch_last_ >= pitch_param1_)
      ei_ = -ki_sat_ / ki_;
    else if(pitch_last_ > pitch_param2_)
      ei_ = -ki_sat_ / ki_ * (pitch_last_ - pitch_param2_) / (pitch_param1_ - pitch_param2_);
    else
      ei_ = 0;

    v_filtered_ = 0;
  }
  else
  {
    ros::Time time_now_ = ros::Time::now();
    ros::Duration time_diff_ = time_now_ - time_pre_;
    double dt_ = time_diff_.toSec();
    double e_now_ = cmd_vel_ - odovel;
    v_filtered_ += (odovel - v_filtered_) * dt_ / tau_v_;

    ei_ += (0.5 * dt_ * (e_pre_ + e_now_));
    if(ki_ * ei_ > ki_sat_)
      ei_ = ki_sat_ / ki_;
    else if(ki_ * ei_ < -ki_sat_)
      ei_ = -ki_sat_ / ki_;

    double u = kp_ * (cmd_vel_ - v_filtered_) + ki_ * ei_;
    u = BOUND(u, -1.0, 1.0);

    if(u > throttle_zero_thres_/coeff_th_)
    {
      throttle.data = coeff_th_ * u;
      brake.data = 0;
    }
    else if(u < -brake_zero_thres_/coeff_bp_)
    {
      throttle.data = 0;
      brake.data = coeff_bp_ * u;
    }
    else
    {
      throttle.data = 0;
      brake.data = 0;
    }

    time_pre_ = time_now_;
    e_pre_ = e_now_;
  }

  throttle_pub_.publish(throttle);
  brakepedal_pub_.publish(brake);
}





int main(int argc, char**argv)
{
  ros::init(argc, argv, "speed_controller");
  PID_Speed pidc;
  ros::spin();
  return 0;
}
