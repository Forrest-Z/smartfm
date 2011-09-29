#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <lse_xsens_mti/imu_rpy.h>

#include <lowlevel/PID.h>
#include <lowlevel/ButtonState.h>



class PID_Speed
{
    public:
        PID_Speed(ros::NodeHandle);

    private:
        ros::NodeHandle n;
        ros::Subscriber cmdVelSub, odoSub, rpySub, buttonSub;
        ros::Publisher throttlePub, brakePedalSub, pidPub;

        void cmdVelCallBack(geometry_msgs::Twist);
        void rpyCallBack(lse_xsens_mti::imu_rpy);
        void odoCallBack(nav_msgs::Odometry);
        void buttonCallBack(lowlevel::ButtonState);

        double kp_, ki_, ki_sat_, coeff_bp_;
        double throttle_zero_thres_, brake_zero_thres_, full_brake_thres_;
        double tau_v_;
        double pitch_param1_, pitch_param2_;

        double cmd_vel_;
        bool emergency;
        ros::Time time_pre_;
        double e_pre_;
        double ei_;
        double v_filtered_;
        double pitch_last_;
};



PID_Speed::PID_Speed(ros::NodeHandle nh) : n(nh)
{
    cmdVelSub= n.subscribe("cmd_vel", 1, &PID_Speed::cmdVelCallBack, this);
    odoSub= n.subscribe("odom", 1, &PID_Speed::odoCallBack, this);
    rpySub = n.subscribe("imu/rpy", 1, &PID_Speed::rpyCallBack, this);
    buttonSub = n.subscribe("button_state", 1, &PID_Speed::buttonCallBack, this);

    throttlePub = n.advertise<std_msgs::Float64>("throttle", 1);
    brakePedalSub = n.advertise<std_msgs::Float64>("brake_angle", 1);
    pidPub = n.advertise<lowlevel::PID>("pid_gain",1);

    ros::NodeHandle private_nh("~");
    if(!private_nh.getParam("kp",kp_)) kp_ = 0.15;
    if(!private_nh.getParam("ki",ki_)) ki_ = 0.2;
    if(!private_nh.getParam("ki_sat",ki_sat_)) ki_sat_ = 0.7;
    if(!private_nh.getParam("coeff_brakepedal",coeff_bp_)) coeff_bp_ = 120;
    if(!private_nh.getParam("throttleZeroThres", throttle_zero_thres_)) throttle_zero_thres_ = 0.1;
    if(!private_nh.getParam("brakeZeroThres", brake_zero_thres_)) brake_zero_thres_ = 5.0;
    if(!private_nh.getParam("fullBrakeThres", full_brake_thres_)) full_brake_thres_ = 0.25;
    if(!private_nh.getParam("tau_v", tau_v_)) tau_v_ = 0.2;
    if(!private_nh.getParam("pitch_param1", pitch_param1_)) pitch_param1_ = 0;
    if(!private_nh.getParam("pitch_param2", pitch_param2_)) pitch_param2_ = -4;

    std::cout <<"kp: " <<kp_ <<" ki: " <<ki_ <<" ki_sat: " <<ki_sat_ <<"\n";
    std::cout <<" coeff_bp: " <<coeff_bp_ <<"\n";
    std::cout <<"throttle_threshold: " <<throttle_zero_thres_;
    std::cout <<" brake_threshold: " <<brake_zero_thres_ <<"\n";
    std::cout <<"tau_v: " <<tau_v_ <<"\n";

    cmd_vel_ = 0;
    time_pre_ = ros::Time::now();
    e_pre_ = 0;
    ei_ = 0;
    v_filtered_ = 0;
    pitch_last_ = 0;
    emergency = false;
}


void PID_Speed::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
    cmd_vel_ = cmd_vel.linear.x;
}


void PID_Speed::rpyCallBack(lse_xsens_mti::imu_rpy rpy)
{
    pitch_last_ = rpy.pitch;
}

void PID_Speed::buttonCallBack(lowlevel::ButtonState bs)
{
    emergency = bs.emergency;
}


void PID_Speed::odoCallBack(nav_msgs::Odometry odom)
{
    std_msgs::Float64 th;
    std_msgs::Float64 bp;
    lowlevel::PID pid;

    double odovel = odom.twist.twist.linear.x;

    if( emergency || (cmd_vel_ <= 0 && odovel <= full_brake_thres_) )
    {
        th.data = 0;
        bp.data = -1 * coeff_bp_;
        cmd_vel_ = 0;
        time_pre_ = ros::Time::now();
        e_pre_ = 0;

        // when starting from a stopped position, initialization is important
        // especially, uphill is tricky
        if(pitch_last_ >= pitch_param1_)
            ei_ = -ki_sat_ / ki_;
        else if(pitch_last_ > pitch_param2_)
            ei_ = -ki_sat_ / ki_ * (pitch_last_ - pitch_param2_) / (pitch_param1_ - pitch_param2_);
        else
            ei_ = 0;

        v_filtered_ = 0;
    }
    else
    {
        ros::Time time_now_ = ros::Time::now();
        ros::Duration time_diff_ = time_now_ - time_pre_;
        double dt_ = time_diff_.toSec();
        double e_now_ = cmd_vel_ - odovel;
        v_filtered_ += (odovel - v_filtered_) * dt_ / tau_v_;

        ei_ += (0.5 * dt_ * (e_pre_ + e_now_));
        if(ki_ * ei_ > ki_sat_)
            ei_ = ki_sat_ / ki_;
        else if(ki_ * ei_ < -ki_sat_)
            ei_ = -ki_sat_ / ki_;

        //added to publish individual gain value
        pid.p_gain = kp_ * (cmd_vel_ - v_filtered_);
        pid.i_gain = ki_ * ei_;
        pid.d_gain= 0;
        pid.v_filter = v_filtered_;
        pidPub.publish(pid);

        double u = pid.p_gain+pid.i_gain;
        if(u > 1.0)
            u = 1.0;
        else if(u < -1.0)
            u = -1.0;

        if(u > throttle_zero_thres_)
        {
            th.data = u;
            bp.data = 0;
        }
        else if(u < -brake_zero_thres_/coeff_bp_)
        {
            th.data = 0;
            bp.data = coeff_bp_ * u;
        }
        else
        {
            th.data = 0;
            bp.data = 0;
        }

        time_pre_ = time_now_;
        e_pre_ = e_now_;
    }

    throttlePub.publish(th);
    brakePedalSub.publish(bp);
}



int main(int argc, char**argv)
{
    ros::init(argc, argv, "speed_controller");
    ros::NodeHandle n;
    PID_Speed pidc(n);
    ros::spin();
    return 0;
}
