/**
 * 
 * Longitudinal control (to be replace to Speed controller)
 * 
 * long_control/PID_Controller.h --> define new massage type for PID controller
 *
**/

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <lse_xsens_mti/imu_rpy.h>

#include <long_control/PID_Msg.h>
#include <phidget_encoders/Encoders.h>

#include <fmutil/fm_math.h>
#include <fmutil/fm_filter.h>

class Parameters
{
    public:
	double kp; ///< Proportional gain
	double kd; ///< Derivative gain
	double ki; ///< Integral gain

	double kp_brake;
	double ki_brake;
	double kd_brake;
	
	double kp_sat; ///< Saturation value of the proportional term
	double ki_sat; ///< Saturation value of the integral term
	double kd_sat; ///< Saturation value of the derivative term

	double kp_sat_brake; ///< Saturation value of the proportional term
	double ki_sat_brake; ///< Saturation value of the integral term
	double kd_sat_brake; ///< Saturation value of the derivative term

	double coeff_bp; ///< Brake angle corresponding to a full brake
	double brake_zero_thres; ///< Only brake if we are below that value
	double throttle_zero_thres; ///< Only throttle if we are above that value
	double full_brake_thres; ///< If the measured velocity is below this value, we consider the car has stopped.
	double full_throttle_thres;
	double throttle_to_brake_thres;
	double throttle_to_neutral_thres;
	double brake_to_throttle_thres;
	double neutral_to_throttle_thres;
	
	double tau_v; ///< Time constant for the velocity filter
	int controller_state; ///0 : Throttle, 1 : Neutral, 2 : Brake
	int controller_state_prev;	///previous state of controller
	double pre_brake_ctrl;
	void getParam();
};

class PID_Controller
{
public:
  PID_Controller();

  virtual double getLookupTable(double desired_vel, double speed_now) = 0;
  virtual double getLookupTable_brake(double delta_vel, double speed_now) = 0;
  virtual void publishCtrlSignal(double throttle, double brake) = 0;
  ros::Publisher throttlePub, brakePedalPub;
  ros::NodeHandle n;
private:
  void bwdDriveCallBack(std_msgs::Bool);
  void cmdVelCallBack(geometry_msgs::Twist);
  void odoCallBack(phidget_encoders::Encoders);
  void emergencyBtnCB(std_msgs::Bool);
  void automodeBtnCB(std_msgs::Bool);
  void safetyBrakeCallBack(std_msgs::Bool);

  ros::Subscriber bwdDriveSub, cmdVelSub, odoSub, emergencyBtnSub, automodeBtnSub, safetyBrakeSub;
  ros::Publisher pidPub;

  Parameters param; ///< parameters of the controller, neatly packed together

  bool bwdDrive; ///< commanded fwd/bwd direction (set by bwdDriveCallBack)
  double cmdVel; ///< The desired velocity (set by cmdVelCallBack)
  bool emergency; ///< is the emergency button pressed (set by emergencyBtnCB)
  bool automode; ///< is the auto mode button pressed (set by automodeBtnCB)
  bool safetyBrake_;

  double e_pre; ///< previous error (velocity) --> used for some kind of filtering of the error term
  double e_diff;

  bool do_brake;
  double e_sum, e_sum_brake, e_now;

  fmutil::LowPassFilter vFilter;
  ros::Time last_time;

};

using namespace std;

void Parameters::getParam()
{
  ros::NodeHandle nh("~");

  nh.param( "kp", kp, 2.5 );
  nh.param( "ki", ki, 0.08 ); //0.007 was ok for Marcelo's
  nh.param( "kd", kd, 0.4 );

  nh.param( "kp_brake", kp_brake, 0.6);
  nh.param( "ki_brake", ki_brake, 0.2);

  nh.param( "kp_sat", kp_sat, 1.0 );
  nh.param( "ki_sat", ki_sat, 0.7 );
  nh.param( "kd_sat", kd_sat, 0.3 );

  nh.param( "kp_sat_brake", kp_sat_brake, 1.0 );
  nh.param( "ki_sat_brake", ki_sat_brake, 0.7 );
  nh.param( "kd_sat_brake", kd_sat_brake, 0.3 );

  nh.param( "coeff_brakepedal", coeff_bp, 120.0 ); // full brake value
  nh.param( "brakeZeroThres", brake_zero_thres, -0.5 ); // if velocity difference less than this value then go to braking state
  nh.param( "throttleZeroThres", throttle_zero_thres, 0.1 ); //if velocity difference larger than this value then go to throttle state
  nh.param( "fullBrakeThres", full_brake_thres, 0.25 ); // if cmdVel = 0.0 and vehicle's speed go below this value, then apply full-brakes
  nh.param( "fullThrottleThres", full_throttle_thres, 1.0);
  nh.param( "brakeToThrottleThres", brake_to_throttle_thres, -20.0);
  nh.param( "tau_v", tau_v, 0.2 );

  ROS_INFO("kp: %lf, ki: %lf, kd: %lf", kp, ki, kd);
  cout <<"kp: " <<kp <<" ki: " <<ki <<" kd: "<<kd<<" ki_sat: " <<ki_sat <<"\n";
  cout <<"kp_brake: "<<kp_brake<<" ki_brake: "<<ki_brake<<endl;
  cout <<"coeff_bp: " <<coeff_bp <<" tau_v: " <<tau_v  <<"\n";
  cout <<"brake_threshold: " <<brake_zero_thres <<"\n";
}


enum controllerState{
  THROTTLE,
  NEUTRAL,
  BRAKE
};

PID_Controller::PID_Controller()
{
    bwdDriveSub = n.subscribe("direction_ctrl", 1, &PID_Controller::bwdDriveCallBack, this);
    cmdVelSub = n.subscribe("cmd_vel", 1, &PID_Controller::cmdVelCallBack, this);
    odoSub = n.subscribe("encoder_odo", 1, &PID_Controller::odoCallBack, this);
    emergencyBtnSub = n.subscribe("button_state_emergency", 1, &PID_Controller::emergencyBtnCB, this);
    automodeBtnSub = n.subscribe("button_state_automode", 1, &PID_Controller::automodeBtnCB, this);
    safetyBrakeSub = n.subscribe("safety_stop", 1, &PID_Controller::safetyBrakeCallBack, this);

    pidPub = n.advertise<long_control::PID_Msg>("pid_term",1);
    param.getParam();

    bwdDrive = false;
    emergency = false;
    automode = false;
    safetyBrake_ = false;

    vFilter = fmutil::LowPassFilter(param.tau_v);

    last_time = ros::Time::now();
    
    param.controller_state = BRAKE;	///Brake
    param.controller_state_prev = BRAKE;	///Brake
    //Initial PID_Controller
    e_pre = 0.0;
    e_now = 0.0;
    
		e_sum = 0.0;    
    e_sum_brake = 0.0; 
		e_diff = 0.0;
		
		
    
}

void PID_Controller::bwdDriveCallBack(std_msgs::Bool msg)
{
    bwdDrive = msg.data;
}

void PID_Controller::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
    //Get desired velocity
    cmdVel = cmd_vel.linear.x;
    //ROS_INFO("Setting the desired velocity to %.2f m/s", cmdVel);
}

void PID_Controller::emergencyBtnCB(std_msgs::Bool msg)
{
    // Would be nice here to define some action when there is a transition from
    // one state to another, e.g. reset the integral term, etc.
    emergency = msg.data;
}

void PID_Controller::automodeBtnCB(std_msgs::Bool msg)
{
    // Would be nice here to define some action when there is a transition from
    // one state to another, e.g. reset the integral term, etc.
    automode = msg.data;
}

void PID_Controller::safetyBrakeCallBack(std_msgs::Bool m)
{
    safetyBrake_ = m.data;
    if( safetyBrake_ ) ROS_INFO("turning ON safety brake");
    else ROS_INFO("turning OFF safety brake");
}


void PID_Controller::odoCallBack(phidget_encoders::Encoders enc)
{
	long_control::PID_Msg pid_msg;

	pid_msg.desired_vel = cmdVel;
	pid_msg.controller_state = param.controller_state;
	
	double odovel = enc.v;
	double time_interval_tmp =  (ros::Time::now() - last_time).toSec();
	double time_bound = 0.1;
	time_interval_tmp = time_interval_tmp<time_bound?time_interval_tmp:time_bound;
	//enc.dt from phidget is not stable, it can have a very large number which brings havok to the vehicle
	enc.dt = time_interval_tmp;
	
	if( emergency || !automode || (cmdVel == 0.0 && fabs(odovel) <= param.full_brake_thres) || safetyBrake_ )
	{
		// reset controller
		e_pre = 0.0;
		e_sum = 0.0;
		e_sum_brake = 0.0;
		pid_msg.vel_err = 0.0;
		vFilter.reset();
		if( safetyBrake_ ) ROS_INFO("safety brake");
		pid_msg.u_brake_ctrl = param.coeff_bp;
		pid_msg.u_ctrl = 0.0;
		
    
	}
	else
	{
	    //Main part of the code here
	    pid_msg.v_filter = vFilter.filter_dt(enc.dt, odovel);
	    pid_msg.vel_err = cmdVel - pid_msg.v_filter;
	    
	    //Logic switching, Throttle or Brake
	    switch (param.controller_state){
	      case THROTTLE:
		do_brake = false;
		if(pid_msg.vel_err < param.brake_zero_thres){
		  param.controller_state_prev = param.controller_state;
		  param.controller_state = BRAKE;
		  do_brake = true;
		}
		if(pid_msg.vel_err < param.throttle_zero_thres){
		  param.controller_state_prev = param.controller_state;
		  param.controller_state = NEUTRAL;
		  do_brake = false;
		}
		break;
	      case NEUTRAL:
		//check originated from which state
		  do_brake = false;
		if(param.controller_state_prev == BRAKE)
		  do_brake = true;
		
		if(pid_msg.vel_err < param.brake_zero_thres){
		  param.controller_state_prev = param.controller_state;
		  param.controller_state = BRAKE;
		  do_brake = true;
		}
		if(pid_msg.vel_err >= param.throttle_zero_thres){
		  bool brake_below_threshold;
		  if(param.brake_to_throttle_thres < 0)
		    brake_below_threshold = param.pre_brake_ctrl >= param.brake_to_throttle_thres;
		  else
		    brake_below_threshold = param.pre_brake_ctrl <= param.brake_to_throttle_thres;
		  if(brake_below_threshold){
		    param.controller_state_prev = param.controller_state;
		    param.controller_state = THROTTLE;
		    do_brake = false;
		  }
		}
		break;
	      case BRAKE:
		do_brake = true;
		if(pid_msg.vel_err >= param.brake_zero_thres){
		  param.controller_state_prev = param.controller_state;
		  param.controller_state = NEUTRAL;
		  do_brake = true;
		}
		if(pid_msg.vel_err >= param.throttle_zero_thres){
		  param.controller_state_prev = param.controller_state;
		  param.controller_state = THROTTLE;
		  do_brake = false;
		}	
		break;
	    }
	    
	  
	  
	  if(!do_brake)
	  {
		  //--------------------------------//
		  //--- Apply Throttle, no Brake ---//
		  //--------------------------------//
		  //Get velocity error
		  double e_now = cmdVel - pid_msg.v_filter;
		  e_sum_brake = 0.0;
	  
		  //Get look-up table
		  pid_msg.table = getLookupTable(cmdVel, pid_msg.v_filter);
		  
		  //P term
		  pid_msg.p_term = fmutil::symbound<double>(param.kp * e_now, param.kp_sat);
		  
		  //I term
		  e_sum = e_sum + (e_now * enc.dt);	//Integral of error
		  e_sum = fmutil::symbound<double>(e_sum, param.ki_sat/param.ki); //an integral wind up
		  pid_msg.i_term = fmutil::symbound<double>(param.ki * e_sum, param.ki_sat);
		  
		  //D term
		  e_diff = (e_now - e_pre) / enc.dt;
		  pid_msg.d_term = fmutil::symbound<double>(param.kd * e_diff, param.kd_sat);
		  
		  //Sum it all up
		  double u =  pid_msg.table + (pid_msg.p_term + pid_msg.i_term + pid_msg.d_term);
		  
		  pid_msg.u_ctrl = fmutil::symbound<double>(u, param.full_throttle_thres); //Throttle command
			  if(pid_msg.u_ctrl < 0.0) pid_msg.u_ctrl = 0.0;
		  pid_msg.u_brake_ctrl = 0.0;	//Brake command
		  //if(param.controller_state == NEUTRAL)
		    //pid_msg.u_ctrl = 0.0;
		  e_pre = e_now;
	  }else{
		  //--------------------------------//
		  //--- Apply Brake, no Throttle ---//
		  //--------------------------------//
		  //Get velocity error
		  double e_now = cmdVel - pid_msg.v_filter;
		  e_sum = 0.0;
		  //Get look-up table
		  pid_msg.table_brake = getLookupTable_brake(e_now, pid_msg.v_filter);
	  
		  //P term
		  pid_msg.p_brake_term = fmutil::symbound<double>(-param.kp_brake * e_now, param.kp_sat_brake);
		  
		  //I term
		  e_sum_brake = e_sum_brake + (e_now * enc.dt);	//Integral of error
		  e_sum_brake = fmutil::symbound<double>(e_sum_brake, param.ki_sat_brake/param.ki_brake);
		  pid_msg.i_brake_term = fmutil::symbound<double>(-param.ki_brake * e_sum_brake, param.ki_sat_brake);
		  
		  //D term
		  e_diff = (e_now - e_pre) / enc.dt;
		  pid_msg.d_brake_term = fmutil::symbound<double>(-param.kd_brake * e_diff, param.kd_sat_brake);
		  
		  double u_brake = pid_msg.table_brake + (pid_msg.p_brake_term + pid_msg.i_brake_term + pid_msg.d_brake_term);
		  pid_msg.u_brake_ctrl = fmutil::symbound<double>(u_brake, param.coeff_bp); //Brake command
		  pid_msg.u_ctrl = 0.0;	//Throttle command
		  e_pre = e_now;
	  }
	}
	std_msgs::Float64 throttle_value, brake_value;
	brake_value.data = pid_msg.u_brake_ctrl;
	throttle_value.data = pid_msg.u_ctrl;
	publishCtrlSignal(pid_msg.u_ctrl, pid_msg.u_brake_ctrl);
	pidPub.publish(pid_msg);
	param.pre_brake_ctrl = pid_msg.u_brake_ctrl;
}

