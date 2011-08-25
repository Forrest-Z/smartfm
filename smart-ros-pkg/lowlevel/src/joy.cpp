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
#define FULL_BRAKE 180

ros::Publisher throttle_pub, steering_pub, brake_pub, lblinker_pub, rblinker_pub;

int counter = 0;

void joyCallBack(joy::Joy joy_msg)
{
  // 3 axes joystick, each axis value ranges from -1.0 to 1.0
  // axes[0] is the lateral axis: positive to the left
  // axes[1] is the longitudinal axis: positive to the front
  // axes[2] is the wheel axis: positive upward.

  if( ++counter<80 )
    return;
  counter = 0;

  float lat = joy_msg.axes[0];
  float lon = joy_msg.axes[1];

  ROS_INFO("Joystick: axes[0] (lat)=%f, axis[1] (lon)=%f, axis[2] (wheel)=%f",
           joy_msg.axes[0], joy_msg.axes[1], joy_msg.axes[2]);

  std_msgs::Float64 V, B;
  V.data = lon>0 ? lon * MAX_VOLT : 0;
  B.data = lon>0 ? 0 : -lon * FULL_BRAKE;
  throttle_pub.publish(V);
  brake_pub.publish(B);

  std_msgs::Float64 W;
  W.data = lat * STEER_ANG_MAX;
  steering_pub.publish(W);

  ROS_INFO("Resulting commands: throttle_volt=%f, brake_angle=%f, steer_angle=%f",
           V.data, B.data, W.data);

  std_msgs::Bool L, R;
  if( W.data > 20 ) {
    L.data = true;
    R.data = false;
  }
  else if( W.data < -20 ) {
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
