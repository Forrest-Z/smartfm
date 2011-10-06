#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <lse_xsens_mti/imu_rpy.h>

#include <lowlevel/PID.h>
#include <lowlevel/ButtonState.h>


class Parameters
{
    public:
        double kp; ///< Proportional gain
        double ki; ///< Integral gain
        double ki_sat; ///< Saturation value of the integral term
        double coeff_bp; ///< Brake angle corresponding to a full brake
        double throttle_zero_thres; ///< Only apply the throttle if the control signal is above that value
        double brake_zero_thres; ///< Only brake if we are above that value
        double full_brake_thres; ///< If the measured velocity is below this value, we consider the car has stopped.
        double tau_v; ///< Time constant for the velocity filter
        double pitch1, pitch2; ///< pitch thresholds

        void getParam();
};


class PID_Speed
{
    public:
        PID_Speed(ros::NodeHandle);

    private:
        void cmdVelCallBack(geometry_msgs::Twist);
        void rpyCallBack(lse_xsens_mti::imu_rpy);
        void odoCallBack(nav_msgs::Odometry);
        void buttonCallBack(lowlevel::ButtonState);

        ros::NodeHandle n;
        ros::Subscriber cmdVelSub, odoSub, rpySub, buttonSub;
        ros::Publisher throttlePub, brakePedalSub, pidPub;

        Parameters param; ///< parameters of the controller, neatly packed together

        double cmdVel; ///< The desired velocity (set by cmdVelCallBack)
        double pitch; ///< current pitch (set by rpyCallBack)
        bool emergency; ///< is the emergency button pressed (set by buttonCallBack)
        bool automode; ///< is the auto mode button pressed (set by buttonCallBack)

        double time_pre; ///<The previous time --> used to compute dt
        double e_pre; ///< previous error (velocity) --> used for some kind of filtering of the error term
        double ei; ///< the integrated error
        double vFiltered; ///< filtered velocity

};


using namespace std;

#define GETP(name,var,val)  if(!nh.getParam(name, var)) var=val

void Parameters::getParam()
{
    ros::NodeHandle nh("~");

    GETP( "kp", kp, 0.15 );
    GETP( "ki", ki, 0.2 );
    GETP( "ki_sat", ki_sat, 0.7 );
    GETP( "coeff_brakepedal", coeff_bp, 120 );
    GETP( "throttleZeroThres", throttle_zero_thres, 0.1 );
    GETP( "brakeZeroThres", brake_zero_thres, 5 );
    GETP( "fullBrakeThres", full_brake_thres, 0.25 );
    GETP( "tau_v", tau_v, 0.2 );
    GETP( "pitch_param1", pitch1, 0 );
    GETP( "pitch_param2", pitch2, -4 );

    cout <<"kp: " <<kp <<" ki: " <<ki <<" ki_sat: " <<ki_sat <<"\n";
    cout <<"coeff_bp: " <<coeff_bp <<" tau_v: " <<tau_v  <<"\n";
    cout <<"throttle_threshold: " <<throttle_zero_thres <<" brake_threshold: " <<brake_zero_thres <<"\n";
    cout <<"pitch1: " <<pitch1 <<" pitch2: " <<pitch2 <<"\n";
}


PID_Speed::PID_Speed(ros::NodeHandle nh) : n(nh)
{
    cmdVelSub = n.subscribe("cmd_vel", 1, &PID_Speed::cmdVelCallBack, this);
    odoSub = n.subscribe("odom", 1, &PID_Speed::odoCallBack, this);
    rpySub = n.subscribe("imu/rpy", 1, &PID_Speed::rpyCallBack, this);
    buttonSub = n.subscribe("button_state", 1, &PID_Speed::buttonCallBack, this);

    throttlePub = n.advertise<std_msgs::Float64>("throttle", 1);
    brakePedalSub = n.advertise<std_msgs::Float64>("brake_angle", 1);
    pidPub = n.advertise<lowlevel::PID>("pid_gain",1);

    param.getParam();

    time_pre = NAN;
    cmdVel = e_pre = ei = vFiltered = pitch = 0;
    emergency = false;
    automode = false;
}


void PID_Speed::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
    cmdVel = cmd_vel.linear.x;
}


void PID_Speed::rpyCallBack(lse_xsens_mti::imu_rpy rpy)
{
    pitch = rpy.pitch;
}

void PID_Speed::buttonCallBack(lowlevel::ButtonState bs)
{
    // Would be nice here to define some action when there is a transition from
    // one state to another, e.g. reset the integral term, etc.
    emergency = bs.emergency;
    automode = bs.automode;
}


void PID_Speed::odoCallBack(nav_msgs::Odometry odom)
{
    std_msgs::Float64 th;
    std_msgs::Float64 bp;
    lowlevel::PID pid;

    double odovel = odom.twist.twist.linear.x;

    if( isnan(time_pre) || emergency || !automode || (cmdVel <= 0 && odovel <= param.full_brake_thres) )
    {
        th.data = 0;
        bp.data = -1 * param.coeff_bp;
        time_pre = ros::Time::now().toSec();
        e_pre = 0;

        // when starting from a stopped position, initialization is important
        // especially, uphill is tricky
        if( pitch >= param.pitch1 )
            ei = -param.ki_sat / param.ki;
        else if( pitch > param.pitch2 )
            ei = -param.ki_sat / param.ki * (pitch - param.pitch2) / (param.pitch1 - param.pitch2);
        else
            ei = 0;

        vFiltered = 0;
    }
    else
    {
        double time_now_ = ros::Time::now().toSec();
        double dt_ = time_now_ - time_pre;
        double e_now_ = cmdVel - odovel;
        vFiltered += (odovel - vFiltered) * dt_ / param.tau_v;

        ei += (0.5 * dt_ * (e_pre + e_now_));
        if(param.ki * ei > param.ki_sat)
            ei = param.ki_sat / param.ki;
        else if(param.ki * ei < -param.ki_sat)
            ei = -param.ki_sat / param.ki;

        //added to publish individual gain value
        pid.p_gain = param.kp * (cmdVel - vFiltered);
        pid.i_gain = param.ki * ei;
        pid.d_gain= 0;
        pid.v_filter = vFiltered;
        pidPub.publish(pid);

        double u = pid.p_gain+pid.i_gain;
        if(u > 1.0)
            u = 1.0;
        else if(u < -1.0)
            u = -1.0;

        if(u > param.throttle_zero_thres)
        {
            th.data = u;
            bp.data = 0;
        }
        else if(u < -param.brake_zero_thres/param.coeff_bp)
        {
            th.data = 0;
            bp.data = param.coeff_bp * u;
        }
        else
        {
            th.data = 0;
            bp.data = 0;
        }

        time_pre = time_now_;
        e_pre = e_now_;
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
