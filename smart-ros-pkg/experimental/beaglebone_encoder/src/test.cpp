#include "eqep.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>

using namespace std;
int main (int argc, char** argv)
{
    ros::init(argc, argv, "encoder_node");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    
    int encoder_no;
    private_nh.param("encoder_number", encoder_no, 2);
    
    if(encoder_no>3){
      cout<<"Only support 3 encoders, please reduce encoder_no"<<endl;
      return -1;
    }
    
    vector<int> encoders;
    encoders.resize(encoder_no);
    vector<eQEP*> eqeps;
    eqeps.resize(encoder_no);
    vector<ros::Publisher> encoder_pubs;
    encoder_pubs.resize(encoder_no);
    int ocd_no;
    double frequency;
    
    private_nh.param("frequency", frequency, 100.0);
    private_nh.param("ocd_no", ocd_no, 3);
    
    // Allocate an instane of 
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
       
      // Query back the period
      stringstream topic_name;
      topic_name << "bb_encoder_"<<encoders[i];
      //setup publisher
      encoder_pubs[i] = nh.advertise<std_msgs::Int64>(topic_name.str(), 1);
      
    }

    ros::Rate r(frequency);    
    // Read position indefintely
    while(ros::ok())
    {
      for(size_t i=0; i<eqeps.size(); i++){
	std_msgs::Int64 count;
	count.data = eqeps[i]->get_position(false);
	encoder_pubs[i].publish(count);
      }
      ros::spinOnce();
      r.sleep();
    }
    
    // Return success
    return 0;
}
