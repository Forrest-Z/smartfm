/*
Reads position of the joystick, translates it into velocity commands, and sends
it to the golfcar_vel channel.

NOTE:
to run the joystick node:
  rosparam set joy_node/dev "/dev/input/js0"
  rosrun joy joy_node
*/

#include <stdio.h>

#include <ros/ros.h>
#include <joy/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>


#define MAX_VOLT 3.3
#define STEER_ANG_MAX (-540)
#define FULL_BRAKE 220

ros::Publisher throttle_pub, steering_pub, brake_pub, lblinker_pub, rblinker_pub;

void joyCallBack(joy::Joy joy_msg)
{
  // 3 axes joystick, each axis value ranges from -1.0 to 1.0
  // axes[0] is the lateral axis: positive to the left
  // axes[1] is the longitudinal axis: positive to the front
  // axes[2] is the wheel axis: positive upward.
  
  float v = joy_msg.axes[1] * joy_msg.axes[2] * MAX_VOLT; //throttle voltage
  float w = joy_msg.axes[0] * STEER_ANG_MAX; //steering angle
  
  ROS_INFO("Joystick: axes[0]=%f, axis[1]=%f, axis[2]=%f",
           joy_msg.axes[0], joy_msg.axes[1], joy_msg.axes[2]);
  ROS_INFO("Voltage=%f, Angle=%f", v, w);
  
  if( v>0 ) {
    std_msgs::Float64 V, B;
    V.data = v;
    B.data = 0;
    throttle_pub.publish(V);
    brake_pub.publish(B);
  }
  else {
    std_msgs::Float64 V, B;
    V.data = 0;
    B.data = FULL_BRAKE;
    throttle_pub.publish(V);
    brake_pub.publish(B);
  }

  std_msgs::Float64 W;
  W.data = w;
  steering_pub.publish(W);

  std_msgs::Bool L, R;
  if( w > 20 ) {
    L.data = true;
    R.data = false;
  }
  else if( w < -20 ) {
    L.data = true;
    R.data = false;
  }
  else {
    L.data = false;
    R.data = false;
  }
  lblinker_pub.publish(L);
  rblinker_pub.publish(R);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_drive");
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("joy", 1000, joyCallBack);
  
  throttle_pub = n.advertise<std_msgs::Float64>("throttle_volt", 1000);
  steering_pub = n.advertise<std_msgs::Float64>("steer_angle", 1000);
  brake_pub = n.advertise<std_msgs::Float64>("brake_angle", 1000);
  lblinker_pub = n.advertise<std_msgs::Bool>("left_blinker", 1000);
  rblinker_pub = n.advertise<std_msgs::Bool>("right_blinker", 1000);

  puts("Reading from Joystick");
  puts("---------------------------");

  ros::spin();

  return 0;
}
