#include <ros/roscpp.h>
#include <std_msgs/Float32.h>

ros::Publisher* pub;
void callback(std_msgs::Float32 data){
	pub->publish(data);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "receiver");
	ros::nodeHandle nh;
	ros::Subscriber sub = nh.subscribe("latency_test", 1, callback);
	pub = new ros::Publisher(nh.advertise<std_msgs::Float32>("latency_test_fb", 1));
	ros::spin();

	return 0;
}
