#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <lse_xsens_mti/imu_rpy.h>

#include <lowlevel/PID.h>
#include <lowlevel/ButtonState.h>

#include <fmutil/fm_math.h>
#include <golfcar_halstreamer/throttle.h>
#include <golfcar_halstreamer/brakepedal.h>
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
	double kdd; //we are switching between PID and PI need another term
        double ei; ///< the integrated error
        double vFiltered; ///< filtered velocity
        double uCtrl; ///< output of the controller

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
    ROS_INFO("kp: %lf, ki: %lf, kd: %lf", kp, ki, kd);
    cout <<"kp: " <<kp <<" ki: " <<ki <<" kd: "<<kd<<" ki_sat: " <<ki_sat <<"\n";
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

//    throttlePub = n.advertise<std_msgs::Float64>("throttle", 1);
//    brakePedalSub = n.advertise<std_msgs::Float64>("brake_angle", 1);
    throttlePub = n.advertise<golfcar_halstreamer::throttle>("golfcar_speed", 1);
    brakePedalSub = n.advertise<golfcar_halstreamer::brakepedal>("golfcar_brake", 1);
    pidPub = n.advertise<lowlevel::PID>("pid_gain",1);

    param.getParam();

    time_pre = NAN;
    cmdVel = e_pre = ei = vFiltered = pitch = uCtrl = 0;
    emergency = false;
    automode = false;
}


void PID_Speed::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
    cmdVel = cmd_vel.linear.x;
    //ROS_INFO("Setting the desired velocity to %.2f m/s", cmdVel);
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
//    std_msgs::Float64 th;
  //  std_msgs::Float64 bp;
	golfcar_halstreamer::throttle th;
    golfcar_halstreamer::brakepedal bp;

    lowlevel::PID pid;

    double odovel = odom.twist.twist.linear.x;
    if( !automode && !emergency) bp.angle=0;
    else
	{

    if( isnan(time_pre) || emergency || !automode || (cmdVel <= 0 && odovel <= param.full_brake_thres) )
    {
        th.volt = 0;
        bp.angle = -1 * param.coeff_bp;
        time_pre = ros::Time::now().toSec();
        e_pre = 0;
        uCtrl = 0;

      

        vFiltered = 0;
    }
    else
    {/*
        double time_now = ros::Time::now().toSec();
        double dt = time_now - time_pre;
        vFiltered += (odovel - vFiltered) * dt / param.tau_v;
        double e_now = cmdVel - vFiltered;
		
		//the output control is u(k)=u(k-1)+k*del(u)
        // Accumulate integral error
        uCtrl += param.ki*(e_now);

        //pid.p_gain = param.kp * e_now;
        pid.i_gain = param.ki*(e_now);
        pid.v_filter = vFiltered;*/

	double time_now = ros::Time::now().toSec();
        double dt = time_now - time_pre;
        vFiltered += (odovel - vFiltered) * dt / param.tau_v;
        double e_now = cmdVel - vFiltered;

        // Accumulate integral error and limit its range
        if( param.ki !=0 ) {
            ei += 0.5 * dt * (e_pre + e_now);
            ei = BOUND(-param.ki_sat / param.ki, ei, param.ki_sat / param.ki);
        }

        pid.p_gain = param.kp * e_now;
        pid.i_gain = param.ki * ei;
        pid.d_gain = kdd * (e_now - e_pre) / dt;
        pid.v_filter = vFiltered;
        pidPub.publish(pid);

        uCtrl = pid.p_gain + pid.i_gain + pid.d_gain;
        uCtrl = BOUND(-1.0, uCtrl, 1.0);
	
        ROS_INFO("Velocity error: %.2f, uCtrl=%.2f", e_now, uCtrl);

        if(uCtrl > param.throttle_zero_thres)
        {
            th.volt = uCtrl*3.33;//2.95+0.35; we can eliminate the constant 0.35 since we are taking into consideration of uCtrl
            bp.angle = 0;
	    kdd = param.kd;
        }
        else if(uCtrl < -param.brake_zero_thres/param.coeff_bp)
        {
            th.volt = 0;
            bp.angle = param.coeff_bp * uCtrl;
	    kdd = 0;
        }
        else
        {
            th.volt = 0;
            //bp.angle = 0;
        }
	//to take care unstable stopped behavior and reset integrator
	if(cmdVel <=0.1) { //sometimes the commanded velocity may not be exactly 0
		th.volt=0;
		ei = 0;
	}

        time_pre = time_now;
        e_pre = e_now;
    }
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