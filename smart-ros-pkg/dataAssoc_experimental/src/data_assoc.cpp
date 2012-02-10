
#include "data_assoc.h"

using namespace std;




data_assoc::data_assoc(int argc, char** argv) 
{
	ROS_INFO("Starting Pedestrian Avoidance ... ");
	
	/// Setting up subsciption 
    ros::NodeHandle nh;
    
    pedSub_ = nh.subscribe("ped_local_frame_vector", 1, &data_assoc::pedClustCallback, this); how to add multiple subscription to same call back ????
    
    ros::NodeHandle n("~");
    
    /// subscribe to
    //n.param("pedestrian_id_file", ped_id_file, string(""));
    //n.param("policy_file", policy_file, string(""));
    //n.param("model_file", model_file, string(""));
    //n.param("simLen", simLen, 100);
    //n.param("simNum", simNum, 100);
    nh.param("use_sim_time", use_sim_time_, false);
    ///add the simLen parameter and correct the coordinate system!

    //subscribe as heartbeat of the robot
    //scanSub_ = nh.subscribe("clock", 1, &data_assoc::scanCallback, this);
    
    /// Setting up publishing
    pedPub_ = nh.advertise<geometry_msgs::Twist>("???",1);



    timer_ = nh.createTimer(ros::Duration(0.5), &data_assoc::??, this);
    ros::spin();
}

data_assoc::~data_assoc()
{
    //cmd.angular.z = 0;
    //cmd.linear.x = 0;
    //cmdPub_.publish(cmd);
}

double dist(geometry_msgs::Point32 A, geometry_msgs::Point32 B)
{
	double distance = sqrt( (A.x -B.x) *(A.x-B.x) + (A.y -B.y) *(A.y-B.y));
	return distance;
}

void data_assoc::pedClustCallback(ped_momdp_sarsop::ped_local_frame_vector ped_local_vector, perception_experimental::clusters cluster_vector)
{

	/// loop over clusters to match with existing lPedInView
	for(int jj=0; jj< lPedInView.size(); jj++)
	{
		double minDist=10000;
		int minID=-1;
		for(int ii=0; ii < cluster_vector.clusters.size(); ii++)
		{
			double currDist = dist(lPedInView[jj].ped_pose, cluster_vector.clusters[ii].centroid);
			if(currDist < minDist)
			{
				minDist = currDist;
				minID = ii;
			}
		}		
		/// if cluster matched, remove from contention
		if(-1 != minID)
		{
			lPedInView[ii].ped_pose = cluster_vector.clusters[minID].centroid;

			/// remove minID element
			if(cluster_vector.clusters.size())
				cluster_vector.clusters.erase(cluster_vector.clusters.begin()+minID);
		}
		
	}
	/// Add remaining clusters as new pedestrians
	/// check with caveat ....
	for(int ii=0; ii< cluster_vector.clusters.size(); ii++)
	{
		If satisfies some criterion
		
		PED_DATA_ASSOC ped;
		ped.id = assign some id; 
		ped.ped_pose = cluster_vector.clusters[ii].centroid;
	}
	
	
}

void data_assoc::publishPed()
{
    is there a new data association pedestrian structure ??
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_assoc");

    data_assoc *data_assoc_node = new data_assoc();//argc, argv);

    ros::spin();
}
