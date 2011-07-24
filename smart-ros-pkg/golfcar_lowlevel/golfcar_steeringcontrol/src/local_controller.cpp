#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "math.h"
#include "golfcar_halstreamer/steering.h"
#include <stdlib.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Path.h>
#include "geometry_msgs/Point.h"
#include <golfcar_halsampler/odo.h>
#define points_num 9
class golfcar_steeringcontrol
{
	public:
	
	ros::Publisher golfcarsteer_pub;
	ros::Publisher stageros;
	

	double distance_threshold;
	double gc_x, gc_y, yaw_feedback;
	bool emergency;
	int point_counts;
	//constructor
	golfcar_steeringcontrol(ros::NodeHandle nh_):n(nh_)
	{
		sub = n.subscribe("cmd_vel", 1000, &golfcar_steeringcontrol::cmdVelCallBack, this);
		sampler_sub = n.subscribe("golfcar_sampler", 1000, &golfcar_steeringcontrol::samplerCallBack, this);
		golfcarsteer_pub = n.advertise<golfcar_halstreamer::steering>("golfcar_steering", 1);
		emergency = false;
		
		
	}
	
	void samplerCallBack(golfcar_halsampler::odo sampler)
	{
		if(sampler.emergency) emergency = true;
		else emergency = false;
	}
	void cmdVelCallBack(geometry_msgs::Twist cmd_vel)
	{
		golfcar_halstreamer::steering st;
		if(!emergency)
		{
			 double wheel_angle = cmd_vel.angular.z/M_PI*180;
			 double steering_angle = -0.0016*pow(wheel_angle,3) - 0.0032*pow(wheel_angle,2)+16.648*wheel_angle-1.3232;
			 if(steering_angle > 540) steering_angle = 540;
			 if(steering_angle < -540) steering_angle = -540;
			 st.angle = -steering_angle;
			 
			
		}
		else{st.angle = 0;}
		
		golfcarsteer_pub.publish(st);
			
	}
	
	private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Subscriber sampler_sub;
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "golfcar_localSteeringControl");
	ros::NodeHandle nh_;
	golfcar_steeringcontrol *control_ = new golfcar_steeringcontrol(nh_);
	
	
	ros::spin();
	
	return 0;
}
	
