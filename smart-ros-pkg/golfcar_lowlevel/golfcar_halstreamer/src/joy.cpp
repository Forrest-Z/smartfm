/*
Reads position of the joystick, translates it into velocity commands, and sends
it to the golfcar_vel channel.
*/

#include <stdio.h>

#include <ros/ros.h>
#include <joy/Joy.h>
#include <golfcar_halstreamer/vel.h>

ros::Publisher chatter_pub;

void joyCallBack(joy::Joy joy_)
{
  golfcar_halstreamer::vel velocity;
  velocity.speed=joy_.axes[1]*joy_.axes[2]*1.5;
  velocity.angle=-joy_.axes[0]*540.0;
  ROS_INFO("Speed=%lf, Angle=%d", velocity.speed, (int)velocity.angle);
  chatter_pub.publish(velocity);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyhook");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joy", 1000, joyCallBack);
  chatter_pub = n.advertise<golfcar_halstreamer::vel>("golfcar_vel", 1000);


  puts("Reading from Joystick");
  puts("---------------------------");

  ros::spin();

  return 0;
}
