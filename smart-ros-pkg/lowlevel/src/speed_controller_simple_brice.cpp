/* todo:
- increase P
- smoother prior transition --> larger thresholds or smoothing term
*/


#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <lse_xsens_mti/imu_rpy.h>

#include <lowlevel/PID.h>
#include <lowlevel/ButtonState.h>
#include <lowlevel/Encoders.h>

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

        double ctrl_period;

        double prior_v0; ///< prior velocity profile: v=(u-prior_v0)*prior_coeff
        double prior_coeff; ///< prior velocity profile: v=(u-prior_v0)*prior_coeff

        void getParam();
};


class PID_Speed
{
    public:
        PID_Speed();
        void control();

    private:
        void cmdVelCallBack(geometry_msgs::Twist);
        void rpyCallBack(lse_xsens_mti::imu_rpy);
        void odoCallBack(lowlevel::Encoders);
        void buttonCallBack(lowlevel::ButtonState);

        void pub(float brake, float throttle);

        ros::NodeHandle n;
        ros::Subscriber cmdVelSub, odoSub, rpySub, buttonSub;
        ros::Publisher throttlePub, brakePedalPub, pidPub;
        lowlevel::PID pid_msg;

        Parameters param; ///< parameters of the controller, neatly packed together

        double cmdVel; ///< The desired velocity (set by cmdVelCallBack)
        double pitch; ///< current pitch (set by rpyCallBack)
        bool emergency; ///< is the emergency button pressed (set by buttonCallBack)
        bool automode; ///< is the auto mode button pressed (set by buttonCallBack)
        double odovel; ///< the current velocity (unfiltered)

        double time_pre; ///<The previous time --> used to compute dt
        double e_pre; ///< previous error (velocity) --> used for some kind of filtering of the error term
        double kdd; ///< we are switching between PID and PI need another term
        double ei; ///< the integrated error
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
    GETP( "coeff_brakepedal", coeff_bp, 120 );
    GETP( "throttleZeroThres", throttle_zero_thres, 0.1 ); //to eliminate the unstable behavior after braking
    GETP( "brakeZeroThres", brake_zero_thres, 5 );
    GETP( "fullBrakeThres", full_brake_thres, 0.25 );
    GETP( "tau_v", tau_v, 0.2 );
    GETP( "pitch_param1", pitch1, 0 );
    GETP( "pitch_param2", pitch2, -4 );
    GETP( "ctrl_period", ctrl_period, 0.1);
    GETP( "prior_v0", prior_v0, 0.1);
    GETP( "prior_coeff", prior_coeff, 3.9);
    ROS_INFO("kp: %lf, ki: %lf, kd: %lf, ki_sat=%lf", kp, ki, kd, ki_sat);
}


PID_Speed::PID_Speed()
{
    cmdVelSub = n.subscribe("cmd_vel", 1, &PID_Speed::cmdVelCallBack, this);
    odoSub = n.subscribe("encoders", 1, &PID_Speed::odoCallBack, this);
    rpySub = n.subscribe("imu/rpy", 1, &PID_Speed::rpyCallBack, this);
    buttonSub = n.subscribe("button_state", 1, &PID_Speed::buttonCallBack, this);

    throttlePub = n.advertise<std_msgs::Float64>("throttle", 1);
    brakePedalPub = n.advertise<std_msgs::Float64>("brake_angle", 1);
    pidPub = n.advertise<lowlevel::PID>("pid_gain", 1);

    param.getParam();

    time_pre = -1;
    cmdVel = e_pre = ei = pitch = 0;
    emergency = false;
    automode = false;
}


void PID_Speed::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
    cmdVel = cmd_vel.linear.x;
    ROS_INFO("Setting the desired velocity to %.2f m/s", cmdVel);
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


void PID_Speed::pub(float brake, float throttle)
{
    std_msgs::Float64 bp;
    std_msgs::Float64 th;
    bp.data = brake;
    th.data = throttle;
    brakePedalPub.publish(bp);
    throttlePub.publish(th);
}


void PID_Speed::odoCallBack(lowlevel::Encoders enc)
{
    odovel = enc.v;
    pid_msg.v_filter += (enc.v - pid_msg.v_filter) * enc.dt / param.tau_v;
    pidPub.publish(pid_msg);
}

#define LIMIT(var,val) var = BOUND(-val,var,val)

void PID_Speed::control()
{
    if( time_pre<0 || emergency || !automode || cmdVel<=0.1 )
    {
        // - time_pre < 0: first call to the function
        // - emergency: no need to set the brakes or what: the arduino takes
        // care of setting the brakes and zeroing the velocity. So we only need
        // to reset the static variables
        // - automode: we need to reset the static variables when control is
        // switched to manual mode.
        // - cmdVel<=0.1: to take care unstable stopped behavior and reset
        // integrator (sometimes the commanded velocity may not be exactly 0)

        time_pre = ros::Time::now().toSec();
        e_pre = ei = pid_msg.p_gain = pid_msg.i_gain = pid_msg.d_gain = pid_msg.u_ctrl = 0;
        pidPub.publish(pid_msg);
    }
    else if( ros::Time::now().toSec() > time_pre + param.ctrl_period )
    {
        double time_now = ros::Time::now().toSec();
        double dt = time_now - time_pre;
        double e_now = cmdVel - pid_msg.v_filter;

        // Accumulate integral error and limit its range
        if( param.ki !=0 )
        {
            ei += 0.5 * dt * (e_pre + e_now);
            LIMIT(ei, param.ki_sat / param.ki);
        }

        pid_msg.p_gain = param.kp * e_now;
        pid_msg.i_gain = param.ki * ei;
        pid_msg.d_gain = kdd * (e_now - e_pre) / dt;

        LIMIT(pid_msg.p_gain, 3.0);
        LIMIT(pid_msg.d_gain, 1.0);
        LIMIT(pid_msg.i_gain, 1.0);

        double pid = pid_msg.p_gain + pid_msg.i_gain + pid_msg.d_gain;

        // Apply a prior except:
        // - if the error is too negative. This happens when we are going way
        //   too fast, (i.e. when going down)
        // - or if the pid term is negative (braking) and the error is positive.
        //   This happens when we reached the bottom of the slope
        pid_msg.prior = cmdVel/param.prior_coeff + param.prior_v0;
        if( e_now < -0.3 || (e_now>0.3 && pid<0) )
            pid_msg.prior = 0;

        pid_msg.u_ctrl = pid_msg.prior + pid;
        LIMIT(pid_msg.u_ctrl, 1.0);

        ROS_INFO("Velocity error: %.2f, uCtrl=%.2f", e_now, pid_msg.u_ctrl);

        float bp=0, th=0;
        if( pid_msg.u_ctrl > param.throttle_zero_thres )
        {
            th = pid_msg.u_ctrl;
            kdd = param.kd;
        }
        else if( pid_msg.u_ctrl < -param.brake_zero_thres/param.coeff_bp )
        {
            bp = -param.coeff_bp * pid_msg.u_ctrl;
            kdd = 0;
        }

        time_pre = time_now;
        e_pre = e_now;

        pidPub.publish(pid_msg);
        pub(bp, th);
    }
}



int main(int argc, char**argv)
{
    ros::init(argc, argv, "speed_controller");
    PID_Speed pidc;

    while( ros::ok() )
    {
        ros::spinOnce();
        pidc.control();
    }
    return 0;
}
