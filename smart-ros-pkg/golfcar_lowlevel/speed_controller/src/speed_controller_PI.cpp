#include <speed_controller/speed_controller_PI.h>

namespace PID_Speed{

  PID_Speed::PID_Speed()
  {
    ros::NodeHandle private_nh_("~");
    ros::NodeHandle n;
    cmd_vel_sub_= n.subscribe("cmd_vel", 1, &PID_Speed::cmdVelCallBack, this);
    sampler_sub_= n.subscribe("golfcar_sampler", 1, &PID_Speed::samplerCallBack, this);
    throttle_pub_ = n.advertise<golfcar_halstreamer::throttle>("golfcar_speed", 1);
    brakepedal_pub_ = n.advertise<golfcar_halstreamer::brakepedal>("golfcar_brake", 1);

    if(!private_nh_.getParam("kp",kp_)) kp_ = 0.3;
    if(!private_nh_.getParam("ki",ki_)) ki_ = 0.3;
    if(!private_nh_.getParam("ki_sat",ki_sat_)) ki_sat_ = 0.4;
    if(!private_nh_.getParam("coeff_throttle",coeff_th_)) coeff_th_ = 3;
    if(!private_nh_.getParam("coeff_brakepedal",coeff_bp_)) coeff_bp_ = 120;
    if(!private_nh_.getParam("throttleZeroThres", throttle_zero_thres_)) throttle_zero_thres_=0.1;
    if(!private_nh_.getParam("brakeZeroThres", brake_zero_thres_)) brake_zero_thres_=1;

    cmd_vel_ = 0;
    time_pre_ = ros::Time::now();
    e_pre_ = 0;
    ei_ = 0;

    std::cout<<"kp: "<<kp_<<" ki: "<<ki_<<" ki_sat: "<<ki_sat_<<"\n";
    std::cout<<"coeff_th: "<<coeff_th_<<" coeff_bp: "<<coeff_bp_<<"\n";
    std::cout<<"throttle_threshold: "<<throttle_zero_thres_<<" brake_threshold: "<<brake_zero_thres_<<"\n";
  }

  void PID_Speed::samplerCallBack(golfcar_halsampler::odo sampler)
  {
    golfcar_halstreamer::throttle th;
    golfcar_halstreamer::brakepedal bp;

    if(sampler.emergency || cmd_vel_ <= 0)
    {
      th.volt = 0; bp.angle = -1 * coeff_bp_;
      cmd_vel_ = 0; time_pre_ = ros::Time::now(); e_pre_ = 0; ei_ = 0;
    }
    else
    {
      ros::Time time_now_ = ros::Time::now();
      ros::Duration time_diff_ = time_now_ - time_pre_;
      double dt_ = time_diff_.toSec();
      double e_now_ = cmd_vel_ - e_pre_;

      ei_ += (0.5 * dt_ * (e_pre_ + e_now_));
      if(ki_ * ei_ > ki_sat_)
	ei_ = ki_sat_ / ki_;
      else if(ki_ * ei_ < -ki_sat_)
	ei_ = -ki_sat_ / ki_;

      double u = kp_ * e_now_ + ki_ * ei_;
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
		
}


int main(int argc, char**argv)
{
  ros::init(argc, argv, "speed_controller");
  PID_Speed::PID_Speed *pidc = new PID_Speed::PID_Speed();
  ros::spin();

  return 0;
}
