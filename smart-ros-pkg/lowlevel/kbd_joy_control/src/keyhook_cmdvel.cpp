/*
Reads velocity commands from the keyboard and sends it to the cmd_vel topic.
*/

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyhook");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  std::cout <<"Reading from keyboard\n";
  std::cout <<"---------------------------\n";
  std::cout <<"Type a desired velocity to move the car.\n";

  for(;;)
  {
    double vel;
    std::cin >> vel;
    geometry_msgs::Twist tw;
    tw.linear.x=vel;
    //velocity.angle=turning*t_factor;
    ROS_INFO("Speed=%lf", tw.linear.x);
    chatter_pub.publish(tw);
  }

  return 0;
}
