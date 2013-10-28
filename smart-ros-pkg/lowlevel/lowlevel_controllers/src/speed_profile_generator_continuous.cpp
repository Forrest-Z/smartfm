/** Speed profile generation.
 *
 * Generates a smooth speed profile: accelerate, maintain, decelerate
 * based on input with given contraint of max_jerk, max_acc, max_vel
 * The given input is target velocity
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

using namespace std;

double max_jerk_acc_, max_acc_;
double max_jerk_dec_, max_dec_;
double target_speed_, target_jerk;
double speed_now_, acc_now_;
double start_time_;
ros::Publisher speed_pub;
int state_now_;

struct StateParameter{
  double t, t_start;
  double v;
  double a;
  double j;
};

struct StateSettings{
  bool acc;
  double time_offset;
  vector<StateParameter> state_parameters;
};

StateSettings stateSettings_;

void printCalculatedStates(double time_now){
  cout.precision(2);
  cout<<stateSettings_.time_offset-start_time_<<": "<<stateSettings_.acc<<endl;
  for(size_t i=0; i<stateSettings_.state_parameters.size(); i++){
    
    cout << "State "<<i<<":"<<"\t";
    cout << "v: "<<stateSettings_.state_parameters[i].v <<"\t";
    cout << "a: "<<stateSettings_.state_parameters[i].a <<"\t";
    cout << "j: "<<stateSettings_.state_parameters[i].j <<"\t";
    cout << "t: "<<stateSettings_.state_parameters[i].t - time_now <<endl;
    cout.precision(0);
  }
}

void targetSpeedCallback(std_msgs::Float64 target_speed){
  if(target_speed_ == target_speed.data) return;
  state_now_ = 0;
  target_speed_ = target_speed.data;
  double cur_time = ros::Time::now().toSec();
  bool full_curve = false;
  
  stateSettings_.time_offset = cur_time;
  //handle the case where acceleration is zero
  //this is the most standard profiles
  int acc_sign = -1;
  stateSettings_.acc = false;
  if(target_speed_ - speed_now_ > 0) {
    acc_sign = 1;
    stateSettings_.acc = true;
  }
  double acc_set_max, jerk_set_max;
  if(acc_now_ == 0){
    if(stateSettings_.acc){
      acc_set_max = max_acc_;
      jerk_set_max = max_jerk_acc_;
    }
    else {
      acc_set_max = max_dec_;
      jerk_set_max = max_jerk_dec_;
    }
    full_curve = fabs(speed_now_ - target_speed_) >= (acc_set_max*acc_set_max)/jerk_set_max;
    
    if(full_curve){
	cout<<"Running full curve"<<endl;
	
	//setup profile parameters
	stateSettings_.state_parameters.resize(4);
	double v_addition = acc_set_max*acc_set_max/(2*jerk_set_max);
	stateSettings_.state_parameters[0].v = speed_now_;
	stateSettings_.state_parameters[0].a = 0;
	stateSettings_.state_parameters[0].j = acc_sign * jerk_set_max;	
	stateSettings_.state_parameters[0].t = cur_time + acc_set_max/jerk_set_max;
	stateSettings_.state_parameters[0].t_start = cur_time;
	
	stateSettings_.state_parameters[1].v = stateSettings_.state_parameters[0].v + acc_sign * v_addition;
	stateSettings_.state_parameters[1].a = acc_sign * acc_set_max;
	stateSettings_.state_parameters[1].j = 0;
	stateSettings_.state_parameters[1].t_start = stateSettings_.state_parameters[0].t;
	
	stateSettings_.state_parameters[2].v = target_speed_ - acc_sign * v_addition;
	stateSettings_.state_parameters[1].t = stateSettings_.state_parameters[0].t + 
	  fabs(stateSettings_.state_parameters[2].v -stateSettings_.state_parameters[1].v)/acc_set_max;
	stateSettings_.state_parameters[2].a = acc_sign * acc_set_max;
	stateSettings_.state_parameters[2].j = - acc_sign * jerk_set_max;
	stateSettings_.state_parameters[2].t = stateSettings_.state_parameters[1].t + acc_set_max/jerk_set_max;
	stateSettings_.state_parameters[2].t_start = stateSettings_.state_parameters[1].t;
	
	stateSettings_.state_parameters[3].v = target_speed_;
	stateSettings_.state_parameters[3].a = 0;
	stateSettings_.state_parameters[3].j = 0;
	//just some time long long into the future
	stateSettings_.state_parameters[3].t = cur_time * 2;
	stateSettings_.state_parameters[3].t_start = stateSettings_.state_parameters[2].t;
	
      }
      else {
	double acc_target = sqrt(fabs(target_speed_ - speed_now_) * jerk_set_max);
	cout<<"Change in speed too small, limitting max acceleration at "<<acc_target<<endl;
	stateSettings_.state_parameters.resize(3);
	double v_addition = acc_target*acc_target/(2*jerk_set_max);
	stateSettings_.state_parameters[0].v = speed_now_;
	stateSettings_.state_parameters[0].a = 0;
	stateSettings_.state_parameters[0].j = acc_sign * jerk_set_max;
	stateSettings_.state_parameters[0].t = cur_time + acc_target/jerk_set_max;
	stateSettings_.state_parameters[0].t_start = cur_time;
	
	stateSettings_.state_parameters[1].v = stateSettings_.state_parameters[0].v + acc_sign * v_addition;
	stateSettings_.state_parameters[1].a = acc_sign * acc_target;
	stateSettings_.state_parameters[1].j = -acc_sign * jerk_set_max;
	stateSettings_.state_parameters[1].t_start = stateSettings_.state_parameters[0].t;
	stateSettings_.state_parameters[1].t = stateSettings_.state_parameters[0].t + acc_target/jerk_set_max;
	
	stateSettings_.state_parameters[2].v = target_speed_;
	stateSettings_.state_parameters[2].a = 0;
	stateSettings_.state_parameters[2].j = 0;
	stateSettings_.state_parameters[2].t_start = stateSettings_.state_parameters[1].t;
	stateSettings_.state_parameters[2].t = cur_time * 2;
      }
      printCalculatedStates(cur_time);
  }
  else {
    cout<<"Unhandled"<<endl;
    return;
  }
  
}

void updateSpeedDynamicModel(double j, double acc0, double speed0, double time_now, double time_offset){
  double t = time_now - time_offset;
  acc_now_ = acc0 + j * t;
  speed_now_ = speed0 + acc0 * t + 0.5 * j * t * t;
  //ROS_INFO_STREAM(speed0<<" "<<acc0<<" "<<t<<" "<<j);
}

void callback(const ros::TimerEvent & event)
{
    double cur_time = event.current_real.toSec();
    double j = 0;
    
    //Check for state change
    if(stateSettings_.state_parameters.size() > 0){
      StateParameter s = stateSettings_.state_parameters[state_now_];
      if(cur_time > s.t) state_now_++;
      updateSpeedDynamicModel(s.j, s.a, s.v, cur_time, s.t_start);
      j = s.j;
    }
    geometry_msgs::Twist tw;
    tw.linear.x = speed_now_;
    tw.linear.y = acc_now_;
    tw.linear.z = j;
    tw.angular.x = state_now_;
    speed_pub.publish(tw);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_profile");
    target_speed_ = 0;
    speed_now_ = acc_now_ = 0;
    state_now_ = 0;
    double frequency;
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    start_time_ = ros::Time::now().toSec();
    speed_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    private_nh.param("max_jerk_acc", max_jerk_acc_, 1.0);
    private_nh.param("max_jerk_dec", max_jerk_dec_, 2.0);
    private_nh.param("max_acc", max_acc_, 0.5);
    private_nh.param("max_dec", max_dec_, 1.0);
    private_nh.param("frequency", frequency, 100.0);
    ros::Timer timer = n.createTimer(ros::Duration(1/frequency), callback);
    ros::Subscriber target_sub = n.subscribe("target_speed", 1, targetSpeedCallback);
    
    ros::spin();
    geometry_msgs::Twist tw;
    tw.linear.x = 0;
    speed_pub.publish(tw);
    return 0;
}
