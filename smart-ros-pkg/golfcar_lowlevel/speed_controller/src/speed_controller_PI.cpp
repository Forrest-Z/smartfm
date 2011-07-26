#include <speed_controller/speed_controller_PI.h>

namespace PID_Speed{

  PID_Speed::PID_Speed()
  {
    ros::NodeHandle n;
    cmd_vel_sub_= n.subscribe("cmd_vel", 1, &PID_Speed::cmdVelCallBack, this);
    sampler_sub_= n.subscribe("golfcar_sampler", 1, &PID_Speed::samplerCallBack, this);
    rpy_sub_ = n.subscribe("imu/rpy", 1, &PID_Speed::rpyCallBack, this);
    throttle_pub_ = n.advertise<golfcar_halstreamer::throttle>("golfcar_speed", 1);
    brakepedal_pub_ = n.advertise<golfcar_halstreamer::brakepedal>("golfcar_brake", 1);

    ros::NodeHandle private_nh("~");
    if(!private_nh.getParam("kp",kp_)) kp_ = 0.15;
    if(!private_nh.getParam("ki",ki_)) ki_ = 0.2;
    if(!private_nh.getParam("ki_sat",ki_sat_)) ki_sat_ = 0.7;
    if(!private_nh.getParam("coeff_throttle",coeff_th_)) coeff_th_ = 3.3;
    if(!private_nh.getParam("coeff_brakepedal",coeff_bp_)) coeff_bp_ = 120;
    if(!private_nh.getParam("throttleZeroThres", throttle_zero_thres_)) throttle_zero_thres_ = 0.3;
    if(!private_nh.getParam("brakeZeroThres", brake_zero_thres_)) brake_zero_thres_ = 5.0;
    if(!private_nh.getParam("fullBrakeThres", full_brake_thres_)) full_brake_thres_ = 0.25;
    if(!private_nh.getParam("tau_v", tau_v_)) tau_v_ = 0.2;
    if(!private_nh.getParam("pitch_param1", pitch_param1_)) pitch_param1_ = 0;
    if(!private_nh.getParam("pitch_param2", pitch_param2_)) pitch_param2_ = -4;

    std::cout<<"kp: "<<kp_<<" ki: "<<ki_<<" ki_sat: "<<ki_sat_<<"\n";
    std::cout<<"coeff_th: "<<coeff_th_<<" coeff_bp: "<<coeff_bp_<<"\n";
    std::cout<<"throttle_threshold: "<<throttle_zero_thres_<<" brake_threshold: "<<brake_zero_thres_<<"\n";
    std::cout<<"tau_v: "<<tau_v_<<"\n";

    cmd_vel_ = 0;
    time_pre_ = ros::Time::now();
    e_pre_ = 0;
    ei_ = 0;
    v_filtered_ = 0;
    pitch_last_ = 0;
  }

  void PID_Speed::samplerCallBack(golfcar_halsampler::odo sampler)
  {
    golfcar_halstreamer::throttle th;
    golfcar_halstreamer::brakepedal bp;

    if(sampler.emergency || (cmd_vel_ <= 0 && sampler.vel <= full_brake_thres_))
    {
      th.volt = 0; bp.angle = -1 * coeff_bp_;
      cmd_vel_ = 0; time_pre_ = ros::Time::now(); e_pre_ = 0;

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
      double e_now_ = cmd_vel_ - sampler.vel;
      v_filtered_ += (sampler.vel - v_filtered_) * dt_ / tau_v_;

      ei_ += (0.5 * dt_ * (e_pre_ + e_now_));
      if(ki_ * ei_ > ki_sat_)
	ei_ = ki_sat_ / ki_;
      else if(ki_ * ei_ < -ki_sat_)
	ei_ = -ki_sat_ / ki_;

      double u = kp_ * (cmd_vel_ - v_filtered_) + ki_ * ei_;
      if(u > 1.0)
	u = 1.0;
      else if(u < -1.0)
	u = -1.0;

      if(u > throttle_zero_thres_/coeff_th_)
      {
	th.volt = coeff_th_ * u;
	bp.angle = 0;
      }
      else if(u < -brake_zero_thres_/coeff_bp_)
      {
	th.volt = 0;
	bp.angle = coeff_bp_ * u;
      }
      else
      {
	th.volt = 0; bp.angle = 0;
      }

      time_pre_ = time_now_;
      e_pre_ = e_now_;
    }

    throttle_pub_.publish(th);
    brakepedal_pub_.publish(bp);
  }
	
  void PID_Speed::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
  {
    cmd_vel_ = cmd_vel.linear.x;
  }

  void PID_Speed::rpyCallBack(lse_xsens_mti::imu_rpy rpy)
  {
    pitch_last_ = rpy.pitch;
  }
}


int main(int argc, char**argv)
{
  ros::init(argc, argv, "speed_controller");
  PID_Speed::PID_Speed *pidc = new PID_Speed::PID_Speed();
  if(!pidc) {
    ROS_ERROR("failed to start the process\n");
    return 1;
  }

  ros::spin();

  return 0;
}
