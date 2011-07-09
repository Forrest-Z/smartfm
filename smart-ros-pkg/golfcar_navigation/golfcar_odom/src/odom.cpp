#include <odom.h>

namespace golfcar_odometry{
//Constructor
        golfcar_odometry::golfcar_odometry(ros::NodeHandle nh_):n(nh_)
	{
	        sampling_rate=200;
		yaw = 0;
		sub = n.subscribe("golfcar_sampler", 1000, &golfcar_odometry::samplerCallBack, this);
		lodo = n.subscribe("pose2D", 1000, &golfcar_odometry::pose2DCallBack, this);
		odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
		
	}
	
	void golfcar_odometry::pose2DCallBack(geometry_msgs::Pose2D pose2d)
	{
		yaw = pose2d.theta;
	}
		
	void golfcar_odometry::samplerCallBack(golfcar_odom::odo sampler)
	{
		
		if(pose_init==0)
		{
			//initialize position as zero and take first reading of yaw_rate
			pose_pre = sampler.pose;
			time_pre = ros::Time::now().toSec();
			speed_pre_ = sampler.vel;
			pose_init++;
			return;
		}
		
		odom.twist.twist.linear.x = sampler.vel;

		//calculate absolute x and y from heading and wheel encoder pose
		double distance = sampler.pose - pose_pre;
		pose_pre = sampler.pose;
		odom.header.frame_id = "odom";
		odom.header.stamp = ros::Time::now();
		odom.pose.pose.position.x += distance * cos(yaw);
		odom.pose.pose.position.y += distance * sin(yaw);
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
		odom.pose.pose.orientation = odom_quat;
		
		static tf::TransformBroadcaster broadcaster_b;
		
		broadcaster_b.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::createQuaternionFromYaw(yaw), tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0)),
				ros::Time::now(),"odom", "base_link"));
		odom_pub.publish(odom);
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
