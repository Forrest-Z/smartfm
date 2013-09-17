#include <ros/roscpp.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "publisher");;
	ros::NodeHandle nh;
	ros::Publisher pub = n.advertise<std_msgs::Float32>("latency_test", 1);
	ros::Rate loop_rate(200);

	int count = -540;
	while(ros::ok()){
		std_msgs::Float32 data;
		data.data = count++;
		pub.publish(data);
		if(count==540) count = -540;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
