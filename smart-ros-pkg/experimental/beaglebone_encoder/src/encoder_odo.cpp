#include "eqep.h"
#include <ros/ros.h>
#include <phidget_encoders/Encoders.h>

using namespace std;
int main (int argc, char** argv)
{
    ros::init(argc, argv, "encoder_node");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    
    int encoder_no = 2;
    
    vector<int> encoders;
    encoders.resize(encoder_no);
    vector<eQEP*> eqeps;
    eqeps.resize(encoder_no);
    
    vector<int32_t> last_counts;
    last_counts.resize(encoder_no);
    
    int ocd_no, pps;
    double frequency, wheel_size, last_pub_time;
    
    private_nh.param("frequency", frequency, 100.0);
    private_nh.param("ocd_no", ocd_no, 3);
    private_nh.param("wheel_size", wheel_size, 1.32212398060626);
    private_nh.param("pulse_per_sec", pps, 2048);
    double distance_per_pulse = wheel_size / pps;
   
    for(size_t i=0; i<encoders.size(); i++){
      stringstream ss, encoder_idx_ss;
      encoder_idx_ss<<"encoder_idx_"<<i;
      private_nh.param(encoder_idx_ss.str(), encoders[i], (int)i);
      ss<<"/sys/devices/ocp."<<ocd_no<<"/4830"<<encoders[i]*2<<"000.epwmss/4830"<<encoders[i]*2<<"180.eqep";
      
      // Allocate a pointer for the new eQEP
      // Reminder: Do not to simply assign the pointer after intialization, lot's of
      // error with eQEP.path when the pointer is deferenced. The memory is
      // corrupted by other variables
      eqeps[i] = new eQEP(ss.str(), eQEP::eQEP_Mode_Absolute);
      if( access( ss.str().c_str(), F_OK ) == -1 ) {
	cout<< "Device not found, please check if encoder "<<encoders[i]<<" existed"<<endl;
	return -1;
      }
      // Set the unit time period to 100,000,000 ns, or 0.1 seconds
      eqeps[i]->set_period(uint64_t(1/frequency*1e12));
      
    }
    //setup publisher
    ros::Publisher encoder_pub = nh.advertise<phidget_encoders::Encoders>("bb_encoder_odo", 1);
      
    last_pub_time = ros::Time::now().toSec()-1/frequency;
    ros::Rate r(frequency);    
    // Read position indefintely
    while(ros::ok())
    {
      phidget_encoders::Encoders enc_msg;
      enc_msg.stamp = ros::Time::now();
      double time_now = enc_msg.stamp.toSec();
      enc_msg.dt = time_now - last_pub_time;
      last_pub_time = time_now;
      int32_t current_count_left = eqeps[0]->get_position(false);
      int32_t current_count_right = eqeps[1]->get_position(false);
      enc_msg.d_count_left = current_count_left;
      enc_msg.d_count_right = current_count_right;
      enc_msg.d_left = (current_count_left - last_counts[0]) * distance_per_pulse;
      enc_msg.d_right = (current_count_right - last_counts[1]) * distance_per_pulse;
      enc_msg.d_dist = (enc_msg.d_left+enc_msg.d_right)/2;
      last_counts[0] = current_count_left;
      last_counts[1] = current_count_right;
      encoder_pub.publish(enc_msg);
      ros::spinOnce();
      r.sleep();
    }
    
    // Return success
    return 0;
}
