#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>

using namespace std;

ros::Publisher *pub; 

void signalCallback(std_msgs::Time t){
  double delay = (ros::Time::now() - t.data).toSec();
  std_msgs::Float64 msg;
  msg.data = delay;
  cout<<delay*1000<<" ms\xd"<<flush;
  pub->publish(msg);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "network_delay_receiver");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("network_delay_test_signal", 1, &signalCallback);
  pub = new ros::Publisher(nh.advertise<std_msgs::Float64>("network_delay",1));
  ros::spin();
  return 0;
}