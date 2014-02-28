#include <ros/ros.h>
#include <std_msgs/Time.h>

using namespace std;

void signalCallback(std_msgs::Time t){
  double delay = (ros::Time::now() - t.data).toSec();
  cout<<delay*100<<endl;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "network_delay_receiver");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("network_delay_test_signal", 1, &signalCallback);
  ros::spin();
  return 0;
}