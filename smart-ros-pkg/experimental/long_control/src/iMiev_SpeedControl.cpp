#include "long_control.h"

class iMievControls : public PID_Controller {
public:
  
  double getLookupTable(double desired_vel, double speed_now){
  double output_sig = 0.0;
  output_sig = 23.465*desired_vel + 65.347;
  if(output_sig < 0.0) output_sig = 0.0;
  if(output_sig > 1000.0) output_sig = 1000;
  return output_sig;
  }
  
  double getLookupTable_brake(double delta_vel, double speed_now){
  double output_sig = 0.0;
  if(speed_now > 5.0)
    output_sig = -1.2226*pow(delta_vel, 3) - 10.655 * pow(delta_vel,2) - 42.699 * delta_vel - 17.362;
  else
    output_sig = -0.459*pow(delta_vel, 3) - 4.4253 * pow(delta_vel,2) - 24.27 * delta_vel + 6.7515;
  
  if(output_sig < 0.0) output_sig = 0.0;
  if(output_sig > 100.0) output_sig = 100.0;
  return output_sig;
  }
};


int main(int argc, char**argv)
{
    ros::init(argc, argv, "iMiev_speedcontrols");
    PID_Controller *pidc = new iMievControls();
    ros::spin();
    return 0;
}