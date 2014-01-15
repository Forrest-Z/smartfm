#include "long_control.h"

class GolfcartControls : public PID_Controller {
public:
  
  double getLookupTable(double desired_vel, double speed_now){
  double output_sig = 0.0;
  output_sig = 0.0024*pow(desired_vel,3) - 0.0265*pow(desired_vel,2) + 0.1758*desired_vel - 0.0112;
  //golfcart won't react when when throttle signal is below 0.1
  if(output_sig < 0.1) output_sig = 0.1;
  if(output_sig < 0.0) output_sig = 0.0;
  return output_sig;
  }
  
  double getLookupTable_brake(double delta_vel, double speed_now){
  double output_sig = 0.0;
  output_sig = 10.802*pow(delta_vel,5) + 85.502*pow(delta_vel,4) + 254.74*pow(delta_vel,3)+ 355.09*pow(delta_vel,2) + 252.65*pow(delta_vel,1) +44.879;
  //the curve starts from 44.879 and go down to negative, bacause delta_vel here will be a negative value
  //this actually take care of the natural rolling friction of the vehicle
  //in golfcart it is about 0.33
  if(output_sig > 0.0) output_sig = 0.0;
  return output_sig;
  }
};


int main(int argc, char**argv)
{
    ros::init(argc, argv, "golfcart_speedcontrols");
    PID_Controller *pidc = new GolfcartControls();
    ros::spin();
    return 0;
}