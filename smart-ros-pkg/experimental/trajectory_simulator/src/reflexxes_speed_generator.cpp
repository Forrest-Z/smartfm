#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <ReflexxesOutputValuesToFile.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

class SpeedGenerator{
  double max_speed_, max_acc_, max_dec_, max_jerk_, frequency_;
  ReflexxesAPI *RML;
  RMLVelocityInputParameters *IP;
  RMLVelocityOutputParameters *OP;
  geometry_msgs::Twist cmd_vel_;
  ros::NodeHandle *nh_;
  ros::Timer *timer_;
  ros::Subscriber cmd_sub_, simple_sub_;
  ros::Publisher control_pub_;
  RMLVelocityFlags Flags_;
public:
  SpeedGenerator(double max_speed, double max_acc, double max_dec, double max_jerk, double frequency, ros::NodeHandle *nh):
  max_speed_(max_speed), max_acc_(max_acc), max_dec_(max_dec), max_jerk_(max_jerk), frequency_(frequency), nh_(nh){
    double sleep_time = 1/frequency;
    RML = new ReflexxesAPI(1, sleep_time);
    IP = new RMLVelocityInputParameters(1);
    OP = new RMLVelocityOutputParameters(1);
    current_filtered_speed_ = 0.0;
    Flags_.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;
    IP->CurrentPositionVector->VecData[0] = 0.0;
    IP->CurrentVelocityVector->VecData[0] = 0.0;
    IP->CurrentAccelerationVector->VecData[0] = 0.0;
    IP->MaxAccelerationVector->VecData[0] = max_acc;
    IP->MaxJerkVector->VecData[0] = max_jerk;
    IP->SelectionVector->VecData[0] = true;
    ros::Timer timer = nh_->createTimer(ros::Duration(sleep_time), &SpeedGenerator::controlCycle, this);
    timer_ = &timer;
    control_pub_ = nh_->advertise<geometry_msgs::Twist>("cmd_generator", 1);
    cmd_sub_ = nh_->subscribe("cmd_vel", 1, &SpeedGenerator::updateReceivedCmd, this);
    simple_sub_ = nh_->subscribe("simple_cmd", 1, &SpeedGenerator::updateReceivedSimpleCmd, this);
    ros::spin();
  }
private:
  double current_filtered_speed_;
  double linear_speed_;
  void controlCycle(const ros::TimerEvent& event){
    IP->TargetVelocityVector->VecData[0] = linear_speed_;
    if(linear_speed_ > current_filtered_speed_)
      IP->MaxAccelerationVector->VecData[0] = max_acc_;
    else
      IP->MaxAccelerationVector->VecData[0] = max_dec_;
    int ResultValue	=	RML->RMLVelocity(*IP, OP, Flags_);
    
    
    *IP->CurrentPositionVector		=	*OP->NewPositionVector		;
    *IP->CurrentVelocityVector		=	*OP->NewVelocityVector		;
    *IP->CurrentAccelerationVector	=	*OP->NewAccelerationVector	;
    
    geometry_msgs::Twist cmd_reflexxes;
    cmd_reflexxes.linear.x = OP->NewVelocityVector->VecData[0];
    current_filtered_speed_ = cmd_reflexxes.linear.x;
    cmd_reflexxes.angular.z = cmd_vel_.angular.z;
    if(ResultValue < 0)
    {
	printf("An error occurred (%d).\n", ResultValue	);
	cmd_reflexxes.linear.x = 0.0;
    }
    control_pub_.publish(cmd_reflexxes);
  }
  
  void updateReceivedCmd(geometry_msgs::Twist cmd){
    cmd_vel_ = cmd;
    linear_speed_ = cmd.linear.x;
  }
  
  void updateReceivedSimpleCmd(std_msgs::Float64 cmd){
    linear_speed_ = cmd.data;
  }
};

  
int main(int argc, char** argv){
  ros::init(argc, argv, "speed_generator");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  double max_speed, max_acc, max_dec, max_jerk, frequency;
  priv_nh.param("max_speed", max_speed, 1.5);
  priv_nh.param("max_acc", max_acc, 1.0);
  priv_nh.param("max_dec", max_dec, 1.0);
  priv_nh.param("max_jerk", max_jerk, 0.5);
  priv_nh.param("frequency", frequency, 100.0);
  SpeedGenerator sg(max_speed, max_acc, max_dec, max_jerk, frequency, &nh);
}