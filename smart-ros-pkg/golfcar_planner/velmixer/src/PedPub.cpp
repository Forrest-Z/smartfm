#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensing_on_road/pedestrian_vision_batch.h>
#include <geometry_msgs/Point32.h>
#include <iostream>
using namespace std;

ros::Publisher ped_pub;
int pedNum;
void pedCallBack(sensing_on_road::pedestrian_vision_batchConstPtr ped_batch)
{
	sensing_on_road::pedestrian_vision_batch ped_batch_output=(*ped_batch);
	int n= (ped_batch->pd_vector.size()<pedNum)?(ped_batch->pd_vector.size()):pedNum;
	ped_batch_output.pd_vector.resize(n);
	for(size_t i=0;i<n;i++)
	{
		cout<<"before "<<ped_batch->pd_vector[i].cluster.centroid.x<<" "<<ped_batch->pd_vector[i].cluster.centroid.y<<endl;
		ped_batch_output.pd_vector[i].cluster.centroid.x= ped_batch->pd_vector[i].cluster.centroid.x-(143.19-52.68);
		ped_batch_output.pd_vector[i].cluster.centroid.y=ped_batch->pd_vector[i].cluster.centroid.y-(121.05-23.11);

		cout<<"after "<<ped_batch_output.pd_vector[i].cluster.centroid.x<<" "<<ped_batch_output.pd_vector[i].cluster.centroid.y<<endl;
	}
	ped_batch_output.header.frame_id="/map";
	ped_pub.publish(ped_batch_output);
	cout<<"ped num "<<pedNum<<endl;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"vel_mixer");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");
    p_nh.param("pedNum", pedNum, 10);
	cout<<"pedNum "<<pedNum<<endl;
	ros::Subscriber ped_sub = nh.subscribe("/golfcart/ped_data_assoc", 1, pedCallBack);
	ped_pub=nh.advertise<sensing_on_road::pedestrian_vision_batch>("/ped_data_assoc",1);
	ros::spin();
}
