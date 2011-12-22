#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <golfcar_halstreamer/steering.h>
#include <golfcar_halsampler/odo.h>


class golfcar_steeringcontrol
{
    public:
        golfcar_steeringcontrol(ros::NodeHandle);

    private:
        void samplerCallBack(golfcar_halsampler::odo sampler);
        void cmdVelCallBack(geometry_msgs::Twist cmd_vel);

        ros::NodeHandle n;
        ros::Subscriber cmdVelSub, samplerSub;
        ros::Publisher golfcarsteer_pub;

        bool emergency;
};


golfcar_steeringcontrol::golfcar_steeringcontrol(ros::NodeHandle nh_):n(nh_)
{
    cmdVelSub = n.subscribe("cmd_vel", 1000, &golfcar_steeringcontrol::cmdVelCallBack, this);
    samplerSub = n.subscribe("golfcar_sampler", 1000, &golfcar_steeringcontrol::samplerCallBack, this);
    golfcarsteer_pub = n.advertise<golfcar_halstreamer::steering>("golfcar_steering", 1);
    emergency = false;
}


void golfcar_steeringcontrol::samplerCallBack(golfcar_halsampler::odo sampler)
{
    emergency = sampler.emergency;
}


void golfcar_steeringcontrol::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
    golfcar_halstreamer::steering st;
    if( ! emergency )
    {
        double wheel_angle = cmd_vel.angular.z/M_PI*180;
        double steering_angle = -0.0016*pow(wheel_angle,3) - 0.0032*pow(wheel_angle,2) + 16.648*wheel_angle - 1.3232;
        if(steering_angle > 540) steering_angle = 540;
        if(steering_angle < -540) steering_angle = -540;
        st.angle = -steering_angle;
    }

    golfcarsteer_pub.publish(st);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "golfcar_localSteeringControl");
    ros::NodeHandle nh_;
    golfcar_steeringcontrol control(nh_);
    ros::spin();
    return 0;
}
