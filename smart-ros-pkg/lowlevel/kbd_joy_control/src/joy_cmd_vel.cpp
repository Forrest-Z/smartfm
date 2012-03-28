/** Reads position of the joystick, translates it into velocity commands, and
 * sends it to the cmd_vel channel.
 */

#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>

ros::Publisher speed_pub;

void joyCallBack(joy::Joy joy_)
{
  geometry_msgs::Twist tw;
  tw.linear.x=joy_.axes[1]*2.5;
  tw.angular.z=joy_.axes[0]*40/180*M_PI;
  //ROS_INFO("Speed=%lf, Steering=%lf, brake=%lf", speed.volt, st.angle, bp.angle);
  speed_pub.publish(tw);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joyhook");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joy", 1000, joyCallBack);
  speed_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  puts("Reading from Joystick");
  puts("---------------------------");

  ros::spin();

  return 0;
}
