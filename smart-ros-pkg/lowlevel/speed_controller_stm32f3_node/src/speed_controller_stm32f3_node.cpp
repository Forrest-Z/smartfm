#include <ros/ros.h>
#include <SerialStream.h>
#include <SerialPort.h>
#include "simpleSerialFraming.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <speed_controller_stm32f3_node/PID_stm32.h>
#include <phidget_encoders/Encoders.h>

using namespace std;
using namespace LibSerial;

SerialStream *serial_port_;
double kp_ = 0.0;
double ki_ = 0.0;
double kd_ = 0.0;
double kp_br_ = 0.0;
double ki_br_ = 0.0;
double kd_br_ = 0.0;
double cmd_linear_x_ = 0.0;
int automode_ = 0;


void updateCmd(){
  uint8_t *packet = (uint8_t*)malloc(128*sizeof(uint8_t));
  int packet_size=0;
  serialAddInt(packet, &packet_size, automode_);
  serialAddDouble(packet, &packet_size, cmd_linear_x_);
  serialAddDouble(packet, &packet_size, kp_);
  serialAddDouble(packet, &packet_size, ki_);
  serialAddDouble(packet, &packet_size, kd_);
  serialAddDouble(packet, &packet_size, kp_br_);
  serialAddDouble(packet, &packet_size, ki_br_);
  serialAddDouble(packet, &packet_size, kd_br_);
  packData(packet, &packet_size);
  serial_port_->write((char*)packet, packet_size);
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stm32f3");
  
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  priv_nh.param("kp", kp_, 1.0);
  priv_nh.param("ki", ki_, 0.1);
  priv_nh.param("kd", kd_, 0.01);
  priv_nh.param("kp_br", kp_br_, 1.0);
  priv_nh.param("ki_br", ki_br_, 0.1);
  priv_nh.param("kd_br", kd_br_, 0.01);
  
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
  ros::Rate loop(1000);
  expected_packet_size =  13 * sizeof(double) + 1 * sizeof(int) + 2 * sizeof(uint32_t);
  while (ros::ok()) {
    while(serial_port_->rdbuf()->in_avail()>0){
      char buffer = serial_port_->rdbuf()->sbumpc();
      if(serialReceive((uint8_t*)&buffer, 1)){
	int data_length = expected_packet_size;
	uint8_t data_received_local[data_length];
	for(int i=0; i<data_length; i++)
	  data_received_local[i] = received_data[i];
	double target_speed, kp, ki, kd;
	double p, i, d, u, feedback_speed, feedback_err;
	double enc1_dt, enc2_dt, dt;
	uint32_t enc1_count, enc2_count;
	int automode;
	unpackDataDouble(data_received_local, &data_length, &dt);
	unpackDataInt(data_received_local, &data_length, &automode);
	unpackDataDouble(data_received_local, &data_length, &target_speed);
	unpackDataDouble(data_received_local, &data_length, &kp);
	unpackDataDouble(data_received_local, &data_length, &ki);
	unpackDataDouble(data_received_local, &data_length, &kd);
	unpackDataDouble(data_received_local, &data_length, &p);
	unpackDataDouble(data_received_local, &data_length, &i);
	unpackDataDouble(data_received_local, &data_length, &d);
	unpackDataDouble(data_received_local, &data_length, &u);
	unpackDataDouble(data_received_local, &data_length, &feedback_speed);
	unpackDataDouble(data_received_local, &data_length, &feedback_err);
	unpackDataDouble(data_received_local, &data_length, &enc1_dt);
	unpackDataDouble(data_received_local, &data_length, &enc2_dt);
	unpackDataUInt32(data_received_local, &data_length, &enc1_count);
	unpackDataUInt32(data_received_local, &data_length, &enc2_count);
	speed_controller_stm32f3_node::PID_stm32 pid;
	pid.automode = automode;
	pid.target_speed = target_speed;
	pid.kp = kp;
	pid.ki = ki;
	pid.kd = kd;
	pid.p = p;
	pid.i = i;
	pid.d = d;
	pid.u = u;
	pid.feedback_speed = feedback_speed;
	pid.feedback_err = feedback_err;
	mc_pub.publish(pid);
	phidget_encoders::Encoders encoder_msg;
	encoder_msg.dt = dt;
	encoder_msg.d_dist = (enc1_dt + enc2_dt)/2.0;
	encoder_msg.v = feedback_speed;
	encoder_msg.d_count_left = enc1_count;
	encoder_msg.d_count_right = enc2_count;
	encoder_pub.publish(encoder_msg);
      }
    }
    loop.sleep();
    ros::spinOnce();
  }
  return 0;
}