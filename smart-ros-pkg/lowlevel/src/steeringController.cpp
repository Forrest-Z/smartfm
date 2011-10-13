#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <lowlevel/ButtonState.h>

#define points_num 9

class SteeringController
{
    public:
        SteeringController(ros::NodeHandle nh_);

    private:
        void buttonStateCB(lowlevel::ButtonState);
        void cmdVelCallBack(geometry_msgs::Twist);

        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Subscriber button_state_sub;
        ros::Publisher golfcarsteer_pub;

        double distance_threshold;
        double gc_x, gc_y, yaw_feedback;
        bool emergency;
        int point_counts;
};



SteeringController::SteeringController(ros::NodeHandle nh_) : n(nh_)
{
    sub = n.subscribe("cmd_vel", 1000, &SteeringController::cmdVelCallBack, this);
    button_state_sub = n.subscribe("button_state", 1000, &SteeringController::buttonStateCB, this);
    golfcarsteer_pub = n.advertise<std_msgs::Float64>("steer_angle", 1);
    emergency = false;
}


void SteeringController::buttonStateCB(lowlevel::ButtonState bstate)
{
    if( bstate.emergency )
        emergency = true;
    else
        emergency = false;
}


void SteeringController::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
    float st = 0;
    if( ! emergency )
    {
        double wheel_angle = cmd_vel.angular.z/M_PI*180;
        double steering_angle = -0.0016*pow(wheel_angle,3) - 0.0032*pow(wheel_angle,2) + 16.648*wheel_angle - 1.3232;
        if(steering_angle > 540) steering_angle = 540;
        if(steering_angle < -540) steering_angle = -540;
        st = -steering_angle;
    }

    std_msgs::Float64 msg;
    msg.data = st;
    golfcarsteer_pub.publish(msg);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "SteeringController");
    ros::NodeHandle nh_;
    SteeringController steeringController(nh_);
    ros::spin();
    return 0;
}
