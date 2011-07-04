#include <odom_imu.h>


namespace golfcar_odometry_imu{
//Constructor
        golfcar_odometry_imu::golfcar_odometry_imu(ros::NodeHandle nh_):n(nh_)
	{
		yaw = 0;
		sub = n.subscribe("golfcar_sampler", 1000, &golfcar_odometry_imu::samplerCallBack, this);
		imu = n.subscribe("imu/data", 1000, &golfcar_odometry_imu::imuCallBack, this);
		lodo = n.subscribe("pose2D", 1000, &golfcar_odometry_imu::pose2DCallBack, this);
		odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
		
		imu_started = false;
		pose_y=0;
		yaw_ = 0;

	}
	
	void golfcar_odometry_imu::imuCallBack(sensor_msgs::Imu imu)
	{
		imuQ = imu.orientation;
		imu_started = true;
		yaw_rate = imu.angular_velocity.z;
	}
	void golfcar_odometry_imu::pose2DCallBack(geometry_msgs::Pose2D pose2d)
        {
                laser_yaw = pose2d.theta;
        }	
	void golfcar_odometry_imu::samplerCallBack(golfcar_odom::odo sampler)
	{
		btScalar pitch, roll, yaw;
		tf::Quaternion qt;
		
		tf::quaternionMsgToTF(imuQ, qt);
		
		btMatrix3x3(qt).getRPY(yaw, pitch, roll);
		
		yaw = - yaw; //the quaternion given out by xsens is weird. It is rotated 90 degree along y-axis
		if(pose_init==0 || !imu_started)
		{
			//initialize position as zero and take first reading of yaw_rate
			pose_pre = sampler.pose;
			yaw_pre = yaw;
			time_pre = ros::Time::now().toSec();
			speed_pre_ = sampler.vel;
			pose_init++;
			if(imu_started) ROS_INFO("Odom start!");
			return;
		}
		
		//calculate vehicle heading
		
		odom.twist.twist.linear.x = sampler.vel;
		
		odom.header.frame_id = "odom";
		odom.header.stamp = ros::Time::now();
		double del_yaw=0;
		//only do integration when the vehicle is moving
		if(sampler.vel>0.05)
			del_yaw = yaw -yaw_pre;
			
		//calculate absolute x and y from heading and wheel encoder pose
		double distance = sampler.pose - pose_pre;
		pose_pre = sampler.pose;
		odom.twist.twist.angular.x = distance;
		btMatrix3x3 btm;		
		btm.setRPY(roll, pitch, yaw_);
		tf::Quaternion qt_temp;
		geometry_msgs::Quaternion imu_temp;
		btm.getRotation(qt_temp);
		tf::quaternionTFToMsg(qt_temp, imu_temp);
		
		
		double r11 = cos(yaw_+del_yaw/2);
		double r21 = sin(yaw_+del_yaw/2);
		double r31 = sin(pitch);
		yaw_ += del_yaw;
		yaw_pre = yaw;
		
		odom.pose.pose.position.x+=distance*r11;
		odom.pose.pose.position.y+=distance*r21;
		odom.pose.pose.position.z+=distance*r31;
		odom.pose.pose.orientation = imu_temp;
		static tf::TransformBroadcaster broadcaster_b;
		;
		broadcaster_b.sendTransform(
			tf::StampedTransform(
				tf::Transform(qt_temp, tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)),
				ros::Time::now(),"odom", "base_link"));
		odom_pub.publish(odom);
	}
	
	
  };
	
	

int main(int argc, char **argv)
{
	ros::init(argc, argv, "golfcar_odom_imu");
	ros::NodeHandle nh_;
	golfcar_odometry_imu::golfcar_odometry_imu *odom_ = new golfcar_odometry_imu::golfcar_odometry_imu(nh_);
	
	odom_->pose_init =0;
	odom_->pose_pre=0.0;
	odom_->pose_zero=0.0;
	
	ros::spin();
	
	return 0;
}
