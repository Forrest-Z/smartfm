#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <golfcar_halsampler/odo.h>
#include <lse_xsens_mti/imu_rpy.h>
#include <golfcar_halstreamer/throttle.h>
#include <golfcar_halstreamer/brakepedal.h>
#include <speed_controller/agg.h>
#include <golfcar_odom/gcSpeedFilter.h>

namespace Speed_agg{
  class Speed_agg
  {
    public:
      Speed_agg();
      ~Speed_agg();

    private:
	ros::Subscriber cmd_vel_sub_;
	ros::Subscriber sampler_sub_;
	ros::Subscriber rpy_sub_;
	ros::Subscriber throttle_sub_;
	ros::Subscriber brakepedal_sub_;
	ros::Publisher agg_pub_;

	void cmdVelCallBack(geometry_msgs::Twist cmd_vel);
	void rpyCallBack(lse_xsens_mti::imu_rpy rpy);
	//void samplerCallBack(golfcar_halsampler::odo sampler);
	void samplerCallBack(golfcar_odom::gcSpeedFilter gcsf);
	void throttleCallBack(golfcar_halstreamer::throttle th);
	void brakeCallBack(golfcar_halstreamer::brakepedal bp);
	
	speed_controller::agg a_;
	int count_;
  };
};

namespace Speed_agg{

	Speed_agg::Speed_agg()
	{
		ros::NodeHandle n;

		cmd_vel_sub_= n.subscribe("cmd_vel", 1, &Speed_agg::cmdVelCallBack, this);
		//sampler_sub_= n.subscribe("golfcar_sampler", 1, &Speed_agg::samplerCallBack, this);
sampler_sub_= n.subscribe("speed_filter", 1, &Speed_agg::samplerCallBack, this);
		rpy_sub_ = n.subscribe("ms/imu/rpy", 1, &Speed_agg::rpyCallBack, this);
		throttle_sub_ = n.subscribe("golfcar_speed", 1, &Speed_agg::throttleCallBack, this);
		brakepedal_sub_ = n.subscribe("golfcar_brake", 1, &Speed_agg::brakeCallBack, this);
		agg_pub_ = n.advertise<speed_controller::agg>("speed_agg", 1);
	
		
	}

	Speed_agg::~Speed_agg()
	{
	}

	void Speed_agg::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
	{
		a_.cmd = cmd_vel.linear.x;
	}

	void Speed_agg::rpyCallBack(lse_xsens_mti::imu_rpy rpy)
	{
		a_.pitch = rpy.pitch;
	}

//	void Speed_agg::samplerCallBack(golfcar_halsampler::odo sampler)
void Speed_agg::samplerCallBack(golfcar_odom::gcSpeedFilter gcsf)
	{
		//a_.speed = sampler.vel;
		a_.speed = gcsf.speed;
		a_.speed_f = gcsf.speedFiltered;
		a_.acc = gcsf.acc;
		a_.acc_f = gcsf.accFiltered;
		//if(count_%10 ==0) 
			agg_pub_.publish(a_);
		count_++;
	}

	void Speed_agg::throttleCallBack(golfcar_halstreamer::throttle th)
	{
		a_.th = th.volt;
	}

	void Speed_agg::brakeCallBack(golfcar_halstreamer::brakepedal bp)
	{
		a_.brake = bp.angle;
	}
};

int main(int argc, char**argv)
{
  ros::init(argc, argv, "speed_agg");
  Speed_agg::Speed_agg *pidc = new Speed_agg::Speed_agg();
  if(!pidc) {
    ROS_ERROR("failed to start the process\n");
    return 1;
  }

  ros::spin();

  return 0;
}

