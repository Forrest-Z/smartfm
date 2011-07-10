#include <speed_controller/speed_controller_PI.h>

namespace PID_Speed{

  PID_Speed::PID_Speed()
  {
    ros::NodeHandle private_nh_("~");
    ros::NodeHandle n;
    cmd_vel_sub_= n.subscribe("cmd_vel", 1, &PID_Speed::cmdVelCallBack, this);
    sampler_sub_= n.subscribe("golfcar_sampler", 1, &PID_Speed::samplerCallBack, this);
    speed_pub_ = n.advertise<golfcar_halstreamer::throttle>("golfcar_speed", 1);
    brake_pub_ = n.advertise<golfcar_halstreamer::brakepedal>("golfcar_brake", 1);
    speed_e_pre_ = speed_e_ =  0;
    cmd_vel_ = 0;
    throttle_ = 0;
    first_switch_=true;
    if(!private_nh_.getParam("K1T",K1T_)) K1T_ = 1;
    if(!private_nh_.getParam("K2T",K2T_)) K2T_ = 0.0;
    if(!private_nh_.getParam("K3T",K3T_)) K3T_ = 0.0;
    if(!private_nh_.getParam("K1B",K1B_)) K1B_ = 50;
    if(!private_nh_.getParam("K2B",K2B_)) K2B_ = 0.0;
    if(!private_nh_.getParam("K3B",K3B_)) K3B_ = 0.0;
    if(!private_nh_.getParam("switchThreshold",SwitchThreshold_)) SwitchThreshold_ = 0.2;
    if(!private_nh_.getParam("throttleZeroThres", throttle_zero_thres_)) throttle_zero_thres_=0.1;
    if(!private_nh_.getParam("brakeZeroThres", brake_zero_thres_)) brake_zero_thres_=1;
    if(!private_nh_.getParam("autoControl", auto_control_)) auto_control_=false;
    std::cout<<"K1T: "<<K1T_<<" K2T: "<<K2T_<<" K3T: "<<K3T_<<"\n";
	std::cout<<"K1B: "<<K1B_<<" K2B: "<<K2B_<<" K3B: "<<K3B_<<"\n";
  }
	void PID_Speed::samplerCallBack(golfcar_halsampler::odo sampler)
	{
		golfcar_halstreamer::throttle th;
		golfcar_halstreamer::brakepedal bp;
		
		if(sampler.emergency)
		{
			th.volt = 0; bp.angle=120;
		}
		else 
		{
			if(cmd_vel_>0)
			{
				th.volt = cmd_vel_;
				bp.angle=0;
			}
			else
			{th.volt = 0; bp.angle=120;}
		}
		
		speed_pub_.publish(th);
		brake_pub_.publish(bp);
	}
	
  void PID_Speed::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
  {
    //update commanded velocity
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
