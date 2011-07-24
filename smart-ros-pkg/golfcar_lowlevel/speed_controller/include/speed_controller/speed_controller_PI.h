#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <golfcar_halsampler/odo.h>
#include <lse_xsens_mti/imu_rpy.h>
#include <golfcar_halstreamer/throttle.h>
#include <golfcar_halstreamer/brakepedal.h>

namespace PID_Speed{
  class PID_Speed
  {
    public:
      PID_Speed();
      ~PID_Speed();

    private:
      ros::Subscriber cmd_vel_sub_;
	  ros::Subscriber sampler_sub_;
      ros::Subscriber rpy_sub_;
      ros::Publisher throttle_pub_;
      ros::Publisher brakepedal_pub_;

      double kp_, ki_, ki_sat_, coeff_th_, coeff_bp_;
      double throttle_zero_thres_, brake_zero_thres_, full_brake_thres_;
      double tau_v_;
      double pitch_param1_, pitch_param2_;

      double cmd_vel_;
      ros::Time time_pre_;
      double e_pre_;
      double ei_;
      double v_filtered_;
      double pitch_last_;

      void cmdVelCallBack(geometry_msgs::Twist cmd_vel);
      void rpyCallBack(lse_xsens_mti::imu_rpy rpy);
      void samplerCallBack(golfcar_halsampler::odo sampler);
  };
};
