/** Speed controller (PID).
 *
 */

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <lse_xsens_mti/imu_rpy.h>

#include <lowlevel_controllers/PID.h>
#include <phidget_encoders/EncoderOdo.h>

#include <fmutil/fm_math.h>
#include <fmutil/fm_filter.h>

class Parameters
{
    public:
        double kp; ///< Proportional gain
        double kd; ///< Derivative gain
        double ki; ///< Integral gain

        double kp_sat; ///< Saturation value of the proportional term
        double ki_sat; ///< Saturation value of the integral term
        double kd_sat; ///< Saturation value of the derivative term

        double coeff_bp; ///< Brake angle corresponding to a full brake
        double throttle_zero_thres; ///< Only apply the throttle if the control signal is above that value
        double brake_zero_thres; ///< Only brake if we are above that value
        double full_brake_thres; ///< If the measured velocity is below this value, we consider the car has stopped.

        double tau_v; ///< Time constant for the velocity filter

        void getParam();
};


class PID_Speed
{
    public:
        PID_Speed();

    private:
        void bwdDriveCallBack(std_msgs::Bool);
        void cmdVelCallBack(geometry_msgs::Twist);
        void odoCallBack(phidget_encoders::EncoderOdo);
        void emergencyBtnCB(std_msgs::Bool);
        void automodeBtnCB(std_msgs::Bool);
        void safetyBrakeCallBack(std_msgs::Bool);

        ros::NodeHandle n;
        ros::Subscriber bwdDriveSub, cmdVelSub, odoSub, emergencyBtnSub, automodeBtnSub, safetyBrakeSub;
        ros::Publisher throttlePub, brakePedalPub, pidPub;

        Parameters param; ///< parameters of the controller, neatly packed together

        bool bwdDrive; ///< commanded fwd/bwd direction (set by bwdDriveCallBack)
        double cmdVel; ///< The desired velocity (set by cmdVelCallBack)
        bool emergency; ///< is the emergency button pressed (set by emergencyBtnCB)
        bool automode; ///< is the auto mode button pressed (set by automodeBtnCB)
        bool safetyBrake_;

        double e_pre; ///< previous error (velocity) --> used for some kind of filtering of the error term
        double kdd; ///<we are switching between PID and PI
        double iTerm; ///< the integral term
        double dgain_pre;

        fmutil::LowPassFilter vFilter;
};


using namespace std;

#define GETP(name,var,val)  if(!nh.getParam(name, var)) var=val

void Parameters::getParam()
{
    ros::NodeHandle nh("~");

    GETP( "kp", kp, 2.5 );
    GETP( "ki", ki, 0.08 ); //0.007 was ok for Marcelo's
    GETP( "kd", kd, 0.4 );

    GETP( "kp_sat", kp_sat, 1.0 );
    GETP( "ki_sat", ki_sat, 0.7 );
    GETP( "kd_sat", kd_sat, 0.3 );

    GETP( "coeff_brakepedal", coeff_bp, 120 ); //120
    GETP( "throttleZeroThres", throttle_zero_thres, 0.1 ); //to eliminate the unstable behavior after braking
    GETP( "brakeZeroThres", brake_zero_thres, 5 );
    GETP( "fullBrakeThres", full_brake_thres, 0.25 );

    GETP( "tau_v", tau_v, 0.2 );

//     ROS_INFO("kp: %lf, ki: %lf, kd: %lf", kp, ki, kd);
//     cout <<"kp: " <<kp <<" ki: " <<ki <<" kd: "<<kd<<" ki_sat: " <<ki_sat <<"\n";
//     cout <<"coeff_bp: " <<coeff_bp <<" tau_v: " <<tau_v  <<"\n";
//     cout <<"throttle_threshold: " <<throttle_zero_thres <<" brake_threshold: " <<brake_zero_thres <<"\n";
}


PID_Speed::PID_Speed()
{
    bwdDriveSub = n.subscribe("direction_ctrl", 1, &PID_Speed::bwdDriveCallBack, this);
    cmdVelSub = n.subscribe("cmd_vel", 1, &PID_Speed::cmdVelCallBack, this);
    odoSub = n.subscribe("encoder_odo", 1, &PID_Speed::odoCallBack, this);
    emergencyBtnSub = n.subscribe("button_state_emergency", 1, &PID_Speed::emergencyBtnCB, this);
    automodeBtnSub = n.subscribe("button_state_automode", 1, &PID_Speed::automodeBtnCB, this);
    safetyBrakeSub = n.subscribe("safety_stop", 1, &PID_Speed::safetyBrakeCallBack, this);

    throttlePub = n.advertise<std_msgs::Float64>("throttle", 1);
    brakePedalPub = n.advertise<std_msgs::Float64>("brake_angle", 1);
    pidPub = n.advertise<lowlevel_controllers::PID>("pid_gain",1);

    param.getParam();

    bwdDrive = false;
    cmdVel = e_pre = iTerm = 0.0;
    emergency = false;
    automode = false;
    safetyBrake_ = false;

    vFilter = fmutil::LowPassFilter(param.tau_v);
}

void PID_Speed::bwdDriveCallBack(std_msgs::Bool msg)
{
    bwdDrive = msg.data;
}

void PID_Speed::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
    cmdVel = cmd_vel.linear.x;
    //ROS_INFO("Setting the desired velocity to %.2f m/s", cmdVel);
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

void PID_Speed::safetyBrakeCallBack(std_msgs::Bool m)
{
    safetyBrake_ = m.data;
    if( safetyBrake_ ) ROS_INFO("turning ON safety brake");
    else ROS_INFO("turning OFF safety brake");
}

void PID_Speed::odoCallBack(phidget_encoders::EncoderOdo enc)
{
    std_msgs::Float64 throttle_msg, brake_msg;
    lowlevel_controllers::PID pid;
    pid.desired_vel = cmdVel;

    double odovel = enc.v;
    if( emergency || !automode || (cmdVel == 0.0 && fabs(odovel) <= param.full_brake_thres) || safetyBrake_ )
    {
        // reset controller
        e_pre = dgain_pre = iTerm = 0.0;
        vFilter.reset();
        if( safetyBrake_ ) ROS_INFO("safety brake");
        brake_msg.data = -param.coeff_bp;
    }
    else
    {
        pid.v_filter = vFilter.filter_dt(enc.dt, odovel);

        double e_now = cmdVel - pid.v_filter;
        pid.p_gain = fmutil::symbound<double>(param.kp * e_now, param.kp_sat);

        // Accumulate integral error and limit its range
        iTerm += param.ki * (e_pre + e_now)/2 * enc.dt;
        iTerm = fmutil::symbound<double>(iTerm, param.ki_sat);
        pid.i_gain = iTerm;

        double dTerm = kdd * (e_now - e_pre) / enc.dt;
        pid.d_gain = fmutil::symbound<double>(dTerm, param.kd_sat);

        // filter out spikes in d_gain
        if( fabs(dgain_pre - pid.d_gain)>0.2 )
            pid.d_gain = dgain_pre;
        dgain_pre = pid.d_gain;

        double u = pid.p_gain + pid.i_gain + pid.d_gain;
        pid.u_ctrl = fmutil::symbound<double>(u, 1.0);

        // change the sign for bwd driving assuming pid gains are same for fwd/bwd
        // this has to be changed if different gains are necessary
        if( bwdDrive )
        {
            pid.p_gain = -pid.p_gain;
            pid.i_gain = -pid.i_gain;
            pid.d_gain = -pid.d_gain;
            pid.u_ctrl = -pid.u_ctrl;
        }

        //ROS_INFO("Velocity error: %.2f, u_ctrl=%.2f", e_now, pid.u_ctrl);

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
        if(fabs(cmdVel) <= 0.1) { //sometimes the commanded velocity may not be exactly 0
            throttle_msg.data = 0;
            iTerm = 0;
        }

        e_pre = e_now;
    }

    throttlePub.publish(throttle_msg);
    brakePedalPub.publish(brake_msg);
    pidPub.publish(pid);
}



int main(int argc, char**argv)
{
    ros::init(argc, argv, "speed_controller");
    PID_Speed pidc;
    ros::spin();
    return 0;
}
