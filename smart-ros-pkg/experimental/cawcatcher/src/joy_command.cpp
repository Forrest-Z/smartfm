#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <cawcatcher/CalPotRead.h>
#include <cawcatcher/AngleComm.h>
ros::Publisher *angle_pub;

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void joy_servo_cb( const sensor_msgs::Joy& joy_msg){
  cawcatcher::AngleComm angle_msg;
  angle_msg.LRoll_com = map(1000 * joy_msg.axes[0], -1000, 1000, -45, 45);
  angle_msg.LPitch_com = map(1000 * joy_msg.axes[3], -1000, 1000, -45, 45);
  angle_msg.RRoll_com = map(1000 * joy_msg.axes[0], -1000, 1000, -45, 45);
  angle_msg.RPitch_com = map(1000 * joy_msg.axes[3], -1000, 1000, -45, 45);
  angle_pub->publish(angle_msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "joyIN");

  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("/joy", 1, joy_servo_cb);

  angle_pub = new ros::Publisher(n.advertise<cawcatcher::AngleComm>("cawcatcherIN_2", 1));

  ros::spin();

  return 0;
}

