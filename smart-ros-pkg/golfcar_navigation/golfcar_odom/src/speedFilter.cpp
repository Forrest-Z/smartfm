#include <speedFilter.h>

namespace golfcar_odometry{
//Constructor
        golfcar_odometry::golfcar_odometry(ros::NodeHandle nh_):n(nh_)
	{
	    sampling_rate=200;
		sub = n.subscribe("golfcar_sampler", 1000, &golfcar_odometry::samplerCallBack, this);
		filter_pub = n.advertise<golfcar_odom::gcSpeedFilter>("speed_filter",1);
		speed_filter_ = new filters::MultiChannelFilterChain<double>("double");
		acc_filter_ = new filters::MultiChannelFilterChain<double>("double");
		 if (!speed_filter_->configure(1, "velocity_filter")) {
			ROS_ERROR("Velocity Filters not configured, %s. (namespace %s)", "velocity_filter",
			n.getNamespace().c_str());
		}
		if (!acc_filter_->configure(1, "acc_filter")) {
			ROS_ERROR("Acceleration Filters not configured, %s. (namespace %s)", "acc_filter",
			n.getNamespace().c_str());
		}
		acc_.push_back(0);
		speed_.push_back(0);
	}
	
		
	void golfcar_odometry::samplerCallBack(golfcar_halsampler::odo sampler)
	{
	        golfcar_odom::gcSpeedFilter gcsf;
		
		if(pose_init==0)
		{
			//initialize position as zero and take first reading of yaw_rate
			pose_pre = sampler.pose;
			time_pre = ros::Time::now().toSec();
			speed_pre_ = sampler.vel;
			pose_init++;
			return;
		}
		speed_[0] = sampler.vel;
		speed_filter_->update(speed_, speed_);
		
		gcsf.speed=sampler.vel;
		gcsf.speedFiltered=speed_[0];
		gcsf.acc=(speed_[0]-speed_pre_)*sampling_rate;
		speed_pre_ = speed_[0];
		acc_[0]=gcsf.acc;
		acc_filter_->update(acc_, acc_);
		gcsf.accFiltered =acc_[0];

		
		filter_pub.publish(gcsf);
	}
	
	
  };
	
	

int main(int argc, char **argv)
{
	ros::init(argc, argv, "golfcar_odom");
	ros::NodeHandle nh_;
	golfcar_odometry::golfcar_odometry *odom_ = new golfcar_odometry::golfcar_odometry(nh_);
	
	odom_->pose_init =0;
	odom_->pose_pre=0.0;
	odom_->pose_zero=0.0;
	
	ros::spin();
	
	return 0;
}
