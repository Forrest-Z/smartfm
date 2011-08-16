/*
Reads position of the joystick, translates it into velocity commands, and sends
it to the cmd_vel channel. Only positive longitudinal velocity is sent...
*/

#include <stdio.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>


ros::Publisher cmd_pub;

void joyCallBack(joy::Joy joy_)
{
  geometry_msgs::Twist cmd;
  if(joy_.axes[1]>0)
    cmd.linear.x = joy_.axes[1]*3;
  else
    cmd.linear.x = 0;

  ROS_INFO("speed cmd = %3.2lf", cmd.linear.x);

  cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joyhook");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joy", 1000, joyCallBack);
  cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  puts("Reading from Joystick");
  puts("---------------------------");
  ros::spin();
  return 0;
}
