#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <lse_xsens_mti/imu_rpy.h>

#include <lowlevel_controllers/PID.h>
#include <phidget_encoders/Encoders.h>

#include <fmutil/fm_math.h>

class Parameters
{
    public:
        double kp; ///< Proportional gain
        double kd; ///< Derivative gain
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
        void imuCallBack(lse_xsens_mti::imu_rpy);
        void odoCallBack(phidget_encoders::Encoders);
        void emergencyBtnCB(std_msgs::Bool);
        void automodeBtnCB(std_msgs::Bool);

        ros::NodeHandle n;
        ros::Subscriber cmdVelSub, odoSub, imuSub, emergencyBtnSub, automodeBtnSub;
        ros::Publisher throttlePub, brakePedalPub, pidPub;

        Parameters param; ///< parameters of the controller, neatly packed together

        double cmdVel; ///< The desired velocity (set by cmdVelCallBack)
        double pitch; ///< current pitch (set by imuCallBack)
        bool emergency; ///< is the emergency button pressed (set by emergencyBtnCB)
        bool automode; ///< is the auto mode button pressed (set by automodeBtnCB)

        double time_pre; ///<The previous time --> used to compute dt
        double e_pre; ///< previous error (velocity) --> used for some kind of filtering of the error term
        double kdd; //we are switching between PID and PI need another term
        double ei; ///< the integrated error
        double vFiltered; ///< filtered velocity
        double dgain_pre;
};


using namespace std;

#define GETP(name,var,val)  if(!nh.getParam(name, var)) var=val

void Parameters::getParam()
{
    ros::NodeHandle nh("~");

    GETP( "kp", kp, 2.5 );
    GETP( "ki", ki, 0.08 ); //0.007 was ok for Marcelo's
    GETP( "kd", kd, 0.4 );
    GETP( "ki_sat", ki_sat, 0.7 );
    GETP( "coeff_brakepedal", coeff_bp, 120 ); //120
    GETP( "throttleZeroThres", throttle_zero_thres, 0.1 ); //to eliminate the unstable behavior after braking
    GETP( "brakeZeroThres", brake_zero_thres, 5 );
    GETP( "fullBrakeThres", full_brake_thres, 0.25 );
    GETP( "tau_v", tau_v, 0.2 );
    GETP( "pitch_param1", pitch1, 0 );
    GETP( "pitch_param2", pitch2, -4 );
    ROS_INFO("kp: %lf, ki: %lf, kd: %lf", kp, ki, kd);
    cout <<"kp: " <<kp <<" ki: " <<ki <<" kd: "<<kd<<" ki_sat: " <<ki_sat <<"\n";
    cout <<"coeff_bp: " <<coeff_bp <<" tau_v: " <<tau_v  <<"\n";
    cout <<"throttle_threshold: " <<throttle_zero_thres <<" brake_threshold: " <<brake_zero_thres <<"\n";
    cout <<"pitch1: " <<pitch1 <<" pitch2: " <<pitch2 <<"\n";
}


PID_Speed::PID_Speed(ros::NodeHandle nh) : n(nh)
{
    cmdVelSub = n.subscribe("cmd_vel", 1, &PID_Speed::cmdVelCallBack, this);
    odoSub = n.subscribe("encoders", 1, &PID_Speed::odoCallBack, this);
    imuSub = n.subscribe("imu/rpy", 1, &PID_Speed::imuCallBack, this);
    emergencyBtnSub = n.subscribe("button_state_emergency", 1, &PID_Speed::emergencyBtnCB, this);
    automodeBtnSub = n.subscribe("button_state_automode", 1, &PID_Speed::automodeBtnCB, this);

    throttlePub = n.advertise<std_msgs::Float64>("throttle", 1);
    brakePedalPub = n.advertise<std_msgs::Float64>("brake_angle", 1);
    pidPub = n.advertise<lowlevel_controllers::PID>("pid_gain",1);

    param.getParam();

    time_pre = NAN;
    cmdVel = e_pre = ei = vFiltered = pitch = 0;
    emergency = false;
    automode = true;
}


void PID_Speed::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
    cmdVel = cmd_vel.linear.x;
    //ROS_INFO("Setting the desired velocity to %.2f m/s", cmdVel);
}


void PID_Speed::imuCallBack(lse_xsens_mti::imu_rpy rpy)
{
    pitch = rpy.pitch;
}

void PID_Speed::emergencyBtnCB(std_msgs::Bool msg)
{
    // Would be nice here to define some action when there is a transition from
    // one state to another, e.g. reset the integral term, etc.
    emergency = msg.data;
}

void PID_Speed::automodeBtnCB(std_msgs::Bool msg)
{
    // Would be nice here to define some action when there is a transition from
    // one state to another, e.g. reset the integral term, etc.
    automode = msg.data;
}

void PID_Speed::odoCallBack(phidget_encoders::Encoders enc)
{
    std_msgs::Float64 throttle_msg, brake_msg;
    lowlevel_controllers::PID pid;

    double odovel = enc.v;
    if( !automode && !emergency)
    {
        brake_msg.data=0;
    }
    else
    {
        if( isnan(time_pre) || emergency || !automode || (cmdVel <= 0 && odovel <= param.full_brake_thres) )
        {
            throttle_msg.data = 0;
            brake_msg.data = -1 * param.coeff_bp;
            time_pre = ros::Time::now().toSec();
            e_pre = 0;
            pid.u_ctrl = 0;
            vFiltered = 0;
            dgain_pre = 0;
        }
        else
        {
            double time_now = ros::Time::now().toSec();
            double dt = time_now - time_pre;
            vFiltered += (odovel - vFiltered) * dt / param.tau_v;
            double e_now = cmdVel - vFiltered;

            // Accumulate integral error and limit its range
            if( param.ki !=0 )
            {
                ei += 0.5 * dt * (e_pre + e_now);
                ei = BOUND(-param.ki_sat / param.ki, ei, param.ki_sat / param.ki);
            }

            pid.p_gain = param.kp * e_now;
            pid.p_gain = BOUND( -0.7, pid.p_gain, 1.0);
            pid.i_gain = param.ki * ei;
            pid.d_gain = kdd * (e_now - e_pre) / dt;
            pid.d_gain = BOUND( -0.3, pid.d_gain, 0.3);
            pid.v_filter = vFiltered;

            // filter out spikes in d_gain
            if( fabs(dgain_pre - pid.d_gain)>0.2 )
                pid.d_gain = dgain_pre;
            dgain_pre = pid.d_gain;

            pid.u_ctrl = pid.p_gain + pid.i_gain + pid.d_gain;
            pid.u_ctrl = BOUND(-1.0, pid.u_ctrl, 1.0);

            ROS_INFO("Velocity error: %.2f, u_ctrl=%.2f", e_now, pid.u_ctrl);

            if(pid.u_ctrl > param.throttle_zero_thres)
            {

                throttle_msg.data = pid.u_ctrl;
                brake_msg.data = 0;
                kdd = param.kd;
            }
            else if(pid.u_ctrl < -param.brake_zero_thres/param.coeff_bp)
            {
                dgain_pre = 0;
                throttle_msg.data = 0;
                brake_msg.data = param.coeff_bp * pid.u_ctrl;
                kdd = 0;
            }
            else
            {
                throttle_msg.data = 0;
                //brake_msg.data = 0;
            }
            //to take care unstable stopped behavior and reset integrator
            if(cmdVel <=0.1) { //sometimes the commanded velocity may not be exactly 0
                throttle_msg.data=0;
                ei = 0;
            }

            time_pre = time_now;
            e_pre = e_now;
        }
    }
    throttlePub.publish(throttle_msg);
    brakePedalPub.publish(brake_msg);
    pidPub.publish(pid);
}



int main(int argc, char**argv)
{
    ros::init(argc, argv, "speed_controller");
    ros::NodeHandle n;
    PID_Speed pidc(n);
    ros::spin();
    return 0;
}
