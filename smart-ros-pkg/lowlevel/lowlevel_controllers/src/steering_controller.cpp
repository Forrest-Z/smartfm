#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <fmutil/fm_math.h>


class SteeringController
{
    public:
        SteeringController(ros::NodeHandle nh_);

    private:
        void emergencyBtnCB(std_msgs::Bool);
        void cmdVelCallBack(geometry_msgs::Twist);

        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Subscriber emergency_btn_sub;
        ros::Publisher steer_pub;

        double distance_threshold;
        double gc_x, gc_y, yaw_feedback;
        bool emergency;
        int point_counts;
};



SteeringController::SteeringController(ros::NodeHandle nh_) : n(nh_)
{
    sub = n.subscribe("cmd_vel", 1000, &SteeringController::cmdVelCallBack, this);
    emergency_btn_sub = n.subscribe("button_state_emergency", 1000, &SteeringController::emergencyBtnCB, this);
    steer_pub = n.advertise<std_msgs::Float64>("steer_angle", 1);
    emergency = false;
}


void SteeringController::emergencyBtnCB(std_msgs::Bool msg)
{
    emergency = msg.data;
}


void SteeringController::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
    float st = 0;
    if( ! emergency )
    {
        double wheel_angle = fmutil::r2d(cmd_vel.angular.z);
        double steering_angle = - 0.0016 * pow(wheel_angle,3)
                                - 0.0032 * pow(wheel_angle,2)
                                + 16.648 * wheel_angle
                                - 1.3232;
        steering_angle = BOUND(-540, steering_angle, 540);
        st = -steering_angle;
    }

    std_msgs::Float64 msg;
    msg.data = st;
    steer_pub.publish(msg);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_controller");
    ros::NodeHandle nh_;
    SteeringController steeringController(nh_);
    ros::spin();
    return 0;
}
