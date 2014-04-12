#include <ros/ros.h>
#include <SerialStream.h>
#include <SerialPort.h>
#include "simpleSerialFraming.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <speed_controller_stm32f3_node/PID_stm32.h>
#include <speed_controller_stm32f3_node/PID_autotune.h>
#include <phidget_encoders/Encoders.h>

using namespace std;
using namespace LibSerial;

SerialStream *serial_port_;
double kp_ = 0.0;
double ki_ = 0.0;
double b_weight_ = 0.0;
double kp_br_ = 0.0;
double ki_br_ = 0.0;
double b_weight_br_ = 0.0;
double cmd_linear_x_ = 0.0;
int automode_ = 0;


void updateCmd(){
  uint8_t *packet = (uint8_t*)malloc(128*sizeof(uint8_t));
  int packet_size=0;
  serialAddInt(packet, &packet_size, automode_);
  serialAddFloat(packet, &packet_size, cmd_linear_x_);
  serialAddFloat(packet, &packet_size, kp_);
  serialAddFloat(packet, &packet_size, ki_);
  serialAddFloat(packet, &packet_size, b_weight_);
  serialAddFloat(packet, &packet_size, kp_br_);
  serialAddFloat(packet, &packet_size, ki_br_);
  serialAddFloat(packet, &packet_size, b_weight_br_);
  packData(packet, &packet_size);
  serial_port_->write((char*)packet, packet_size);
//   cout<<"Packet sent with "<<automode_<<" cmd:"<<cmd_linear_x_<<endl;
  free(packet);
}

void automodeCallback(std_msgs::Bool auto_mode){
  automode_  = auto_mode.data;
  updateCmd();
}

void cmdSpeedCallback(geometry_msgs::Twist cmd){
  cmd_linear_x_ = cmd.linear.x;
  updateCmd();
}

enum testState{
  PREPARE, STEP1, STEP2, COLD
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stm32f3");
  
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  priv_nh.param("kp", kp_, 0.66);
  priv_nh.param("ki", ki_, 0.464475);
  priv_nh.param("b_weight", b_weight_, 0.1);
  priv_nh.param("kp_br", kp_br_, .15);
  priv_nh.param("ki_br", ki_br_, 0.2);
  priv_nh.param("b_weight_br", b_weight_br_, 1.0);
  
  serial_port_ = new SerialStream(string("/dev/ttyACM0"), SerialStreamBuf::BAUD_19200,
			   SerialStreamBuf::CHAR_SIZE_8, SerialStreamBuf::PARITY_NONE, 1, SerialStreamBuf::FLOW_CONTROL_NONE);

  //Error management for serial port
  if ( ! serial_port_->IsOpen() ) {
      cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
		<< "Error: Could not open serial port." 
		<< endl ;
      exit(1) ;
  }
  
  ros::Subscriber cmd_sub = n.subscribe("cmd_generator", 1, &cmdSpeedCallback);
  ros::Subscriber auto_sub = n.subscribe("automode", 1, &automodeCallback);
  ros::Publisher mc_pub = n.advertise<speed_controller_stm32f3_node::PID_stm32>("pid_stm32f3", 1);
  ros::Publisher encoder_pub = n.advertise<phidget_encoders::Encoders>("encoder_odo", 1);
  ros::Publisher autotune_pub = n.advertise<speed_controller_stm32f3_node::PID_autotune>("autotune_result", 1);
  ros::Publisher brake_pub = n.advertise<std_msgs::Float64>("golfcart/brake_angle", 1);
  ros::Rate loop(1000);
  expected_packet_size =  13 * sizeof(float) + 3 * sizeof(int) + 2 * sizeof(uint32_t);
  
  //autotune!
  int frequency = 50;
  int count_prepare = 3 * frequency;
  int count_step1 = 10 * frequency + count_prepare;
  int count_step2 = 5 * frequency + count_step1;
  int count_cold = 8 * frequency + count_step2;
  vector<double> kp_test, ki_test, kd_test;
  for(double k_temp = 0.01; k_temp <= 0.1; k_temp += 0.01){
    kp_test.push_back(k_temp);
    ki_test.push_back(0.0);
    kd_test.push_back(0.0);
  }
  int count_test = 0;
  int count_step = 0;
  double abs_error = 0.0;
  double max_overshoot = 0.0;
  double min_u = 0.0;
  double max_u = 0.0;
  testState test_state = PREPARE;
  
  while (ros::ok()) {
    while(serial_port_->rdbuf()->in_avail()>0){
      char buffer = serial_port_->rdbuf()->sbumpc();
      if(serialReceive((uint8_t*)&buffer, 1)){
	int data_length = expected_packet_size;
	uint8_t data_received_local[data_length];
	for(int i=0; i<data_length; i++)
	  data_received_local[i] = received_data[i];
	float target_speed, kp, ki;
	float p, i, u, feedback_speed, feedback_err, filtered_speed;
	float enc1_dt, enc2_dt, dt, b_weight;
	uint32_t enc1_count, enc2_count;
	int automode, serial_received, throttle_state;
	unpackDataFloat(data_received_local, &data_length, &dt);
	unpackDataInt(data_received_local, &data_length, &automode);
	unpackDataInt(data_received_local, &data_length, &throttle_state);
	unpackDataFloat(data_received_local, &data_length, &target_speed);
	unpackDataFloat(data_received_local, &data_length, &kp);
	unpackDataFloat(data_received_local, &data_length, &ki);
	unpackDataFloat(data_received_local, &data_length, &b_weight);
	unpackDataFloat(data_received_local, &data_length, &p);
	unpackDataFloat(data_received_local, &data_length, &i);
	unpackDataFloat(data_received_local, &data_length, &u);
	unpackDataFloat(data_received_local, &data_length, &feedback_speed);
	unpackDataFloat(data_received_local, &data_length, &filtered_speed);
	unpackDataFloat(data_received_local, &data_length, &feedback_err);
	unpackDataFloat(data_received_local, &data_length, &enc1_dt);
	unpackDataFloat(data_received_local, &data_length, &enc2_dt);
	unpackDataUInt32(data_received_local, &data_length, &enc1_count);
	unpackDataUInt32(data_received_local, &data_length, &enc2_count);
	unpackDataInt(data_received_local, &data_length, &serial_received);
	speed_controller_stm32f3_node::PID_stm32 pid;
	pid.automode = automode;
	pid.target_speed = target_speed;
	pid.throttle_state = throttle_state;
	pid.kp = kp;
	pid.ki = ki;
	pid.p = p;
	pid.i = i;
	pid.u = u;
	pid.b_weight = b_weight;
	pid.feedback_speed = feedback_speed;
	pid.filtered_speed = filtered_speed;
	pid.feedback_err = feedback_err;
	pid.serial_received = serial_received;
	mc_pub.publish(pid);
	phidget_encoders::Encoders encoder_msg;
	encoder_msg.dt = dt;
	encoder_msg.d_dist = (enc1_dt + enc2_dt)/2.0;
	encoder_msg.v = feedback_speed;
	encoder_msg.d_count_left = enc1_count;
	encoder_msg.d_count_right = enc2_count;
	encoder_pub.publish(encoder_msg);
	
	std_msgs::Float64 brake_angle;
	brake_angle.data = 0.0;
	if(u<0.0)
	  brake_angle.data = u*90.0;
	brake_pub.publish(brake_angle);
	
	
	//start test count
	if(count_test >= (int)kp_test.size()) {
	  cout<<"Test ended!"<<endl;
	  exit(0);
	} else {
	  switch (test_state) {
	    case PREPARE:
	      //kp=0.2 ki=0.2861952862 no load tuned
	      //kp-0.75 ki=0.4901960784 load tuned
	      kp_ = /*0.75*/0.2;;//0.66;
	      ki_ = /*0.4901960784*/0.2861952862;//1.53;//0.464475;
	      b_weight_ = 0.1;//0.008;
	      kp_br_ = 0.1;//kp_;//kp_test[count_test];
	      ki_br_ = 0.25;//ki_;//ki_test[count_test];
	      b_weight_br_ = 1.0;//kd_test[count_test];
	      cmd_linear_x_ = 0.0;
	      automode_ = 0;
	      updateCmd();
	      if(count_step++ > count_prepare){
		test_state = STEP1;
		automode_ = 1;
	      }
	      break;
	    case STEP1:
	      cmd_linear_x_ = 1.5;
	      updateCmd();
	      abs_error += fabs(feedback_err);
	      if(max_overshoot < feedback_speed) max_overshoot = feedback_speed;
	      if(min_u > u) min_u = u;
	      if(max_u < u) max_u = u;
	      if(count_step++ > count_step1)
		test_state = STEP2;
	      break;
	    case STEP2:
	      cmd_linear_x_ = 1.2;
	      updateCmd();
	      abs_error += fabs(feedback_err);
	      if(min_u > u) min_u = u;
	      if(max_u < u) max_u = u;
	      if(count_step++ > count_step2)
		test_state = COLD;
	      break;
	    case COLD:
	      cmd_linear_x_ = 0.0;
	      
	      updateCmd();
	      abs_error += fabs(feedback_err);
	      if(min_u > u) min_u = u;
	      if(max_u < u) max_u = u;
	      if(count_step++ > count_cold){
		test_state = PREPARE;
		cout<<"Done test "<<count_test<<" kp="<<kp_<<" ki="<<ki_<<": result err="<<abs_error<<" max overshoot="<<max_overshoot<<endl;
		speed_controller_stm32f3_node::PID_autotune result_msg;
		result_msg.kp = kp_;
		result_msg.ki = ki_;
		result_msg.abs_err = abs_error;
		result_msg.max_speed = max_overshoot;
		result_msg.min_input = min_u;
		result_msg.max_input = max_u;
		autotune_pub.publish(result_msg);
		abs_error = 0.0;
		max_overshoot = 0.0;
		count_test++;
		count_step=0;
	      }
	      break;
	  }
	}
      }
    }
    loop.sleep();
    ros::spinOnce();
  }
  return 0;
}