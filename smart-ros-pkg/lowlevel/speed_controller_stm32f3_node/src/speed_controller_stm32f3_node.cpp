#include <ros/ros.h>
#include <SerialStream.h>
#include <SerialPort.h>
#include "simpleSerialFraming.h"
using namespace std;
using namespace LibSerial;
  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stm32f3");
  
  ros::NodeHandle n;
  
  ros::Rate loop_rate(100);
  double small_speed = 0.1;
  
//   double data = 342.34;
//   stringstream small_speed_ss;
//   small_speed_ss<<data;
//   vector<uint8_t> sumsum;
//   uint16_t csum1 = fletcher16((uint8_t *)small_speed_ss.str().c_str(), small_speed_ss.str().size(), sumsum);
//   cout<<"Uint16 output: "<<csum1<<" for "<<small_speed_ss.str()<<endl;
//   cout<<"sum1 "<<(int)sumsum[0]<<" sum2 "<<(int)sumsum[1]<<endl;
  SerialStream serial_port(string("/dev/ttyACM0"), SerialStreamBuf::BAUD_19200,
			   SerialStreamBuf::CHAR_SIZE_8, SerialStreamBuf::PARITY_NONE, 1, SerialStreamBuf::FLOW_CONTROL_NONE);
//   serial_port.Open();

  //Error management for serial port
  if ( ! serial_port.IsOpen() ) {
      cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
		<< "Error: Could not open serial port." 
		<< endl ;
      exit(1) ;
  }
  // while( serial_port.rdbuf()->in_avail() > 0  ) 
  double cmd_speed = 0.0;
  while (ros::ok()) {
    cmd_speed+=0.01;
    double check_no = sin(cmd_speed);
    cout<<"Looping "<<check_no<<endl;
    uint8_t *packet = (uint8_t*)malloc(128*sizeof(uint8_t));
    int packet_size=0;
    serialAdd(packet, &packet_size, check_no);
    packData(packet, &packet_size);
    
//     cout<<"New packet size "<<packet_size<<endl;
//     for(size_t i=0; i<packet_size; i++){
//       cout<<(int)packet[i]<<" ";
//     }
//     cout<<endl;
    //for(int i=0; i<packet_size; i++)
      serial_port.write((char*)packet, packet_size);
    free(packet);

    while(serial_port.rdbuf()->in_avail()>0){
      char buffer = serial_port.rdbuf()->sbumpc();
      if(serialReceive((uint8_t*)&buffer, 1)){
	int data_length = 3 * sizeof(double);
	uint8_t data_received_local[data_length];
	for(int i=0; i<data_length; i++)
	  data_received_local[i] = received_data[i];
	double target_speed, encoder1_speed, encoder2_speed;
	unpackData(data_received_local, &data_length, &target_speed);
	unpackData(data_received_local, &data_length, &encoder1_speed);
	unpackData(data_received_local, &data_length, &encoder2_speed);
	cout<<target_speed<<" "<<encoder1_speed<<" "<<encoder2_speed<<endl;
      }
    }
    loop_rate.sleep();
  }
  return 0;
}