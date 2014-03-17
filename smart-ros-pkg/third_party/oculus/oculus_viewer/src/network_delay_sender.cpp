#include <ros/ros.h>
#include <std_msgs/Time.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "network_delay_sender");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Time>("network_delay_test_signal", 1);
  ros::Rate loop_rate(2);
  while(ros::ok()){
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    pub.publish(msg);
    loop_rate.sleep();
  }
  return 0;
}