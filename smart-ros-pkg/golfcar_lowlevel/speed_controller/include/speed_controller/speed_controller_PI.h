#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <golfcar_halsampler/odo.h>
#include <golfcar_halstreamer/throttle.h>
#include <golfcar_halstreamer/brakepedal.h>
namespace PID_Speed{
  class PID_Speed
  {
    public:
      PID_Speed();
      ~PID_Speed();

    private:
      ros::Subscriber filter_sub_;
      ros::Subscriber cmd_vel_sub_;
      ros::Publisher speed_pub_;
      ros::Publisher brake_pub_;
	  ros::Subscriber sampler_sub_;
	bool first_switch_;
      double cmd_vel_;
      double throttle_,brake_;
      double speed_e_pre2_;
       double speed_e_pre_;
       double speed_e_;
       bool emergency_;
      bool started_;
      bool auto_control_;
      double K1T_, K2T_, K3T_, SwitchThreshold_;
      double K1B_, K2B_, K3B_;
      void cmdVelCallBack(geometry_msgs::Twist cmd_vel);
      void samplerCallBack(golfcar_halsampler::odo sampler);
      double throttle_zero_thres_, brake_zero_thres_;
      std::vector<int> path_pre;
  };
};
