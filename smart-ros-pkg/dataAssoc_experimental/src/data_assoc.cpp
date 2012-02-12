
#include "data_assoc.h"

using namespace std;




data_assoc::data_assoc(int argc, char** argv) 
{
	ROS_INFO("Starting Pedestrian Avoidance ... ");
	
	/// Setting up subsciption 
    ros::NodeHandle nh;
    
    pedClustSub_ = nh.subscribe("ped_laser_cluster", 1, &data_assoc::pedClustCallback, this); 
    pedVisionSub_ = nh.subscribe("ped_vision", 1, &data_assoc::pedVisionCallback, this); 
    /// TBP : how to add multiple subscription to same call back ????
    
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
    pedPub_ = nh.advertise<dataAssoc_experimental::PedDataAssoc>("ped_data_assoc",1); /// topic name
    visualizer_ = nh.advertise<sensor_msgs::PointCloud>("ped_data_assoc_visual",1);
	latest_id=0;

    //timer_ = nh.createTimer(ros::Duration(0.5), &data_assoc::??, this); /// control loop ??
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
	double distance = -1;
	//if(A.x || B.x || A.y || B.y)
	 distance = sqrt( (A.x -B.x) *(A.x-B.x) + (A.y -B.y) *(A.y-B.y));
	 
	return distance;
}

void data_assoc::pedVisionCallback(sensing_on_road::pedestrian_vision_batch pedestrian_vision_vector)
{
	cout << " Entering vision call back with lPedInView " << lPedInView.size() << endl;
	//pedestrian_vision_vector.pd_vector[].cluster.centroid;
	/// loop over clusters to match with existing lPedInView
	for(int jj=0; jj< lPedInView.size(); jj++)
	{
		double minDist=10000;
		int minID=-1;
		for(int ii=0; ii < pedestrian_vision_vector.pd_vector.size(); ii++)
		{
			double currDist = dist(lPedInView[jj].ped_pose, pedestrian_vision_vector.pd_vector[ii].cluster.centroid);
			if( (currDist < minDist) && currDist>-1)
			{
				minDist = currDist;
				minID = ii;
			}
		}		
		//maybe we can add image features to check similarity
		if(minDist < NN_MATCH_THRESHOLD)
		{
			/// if cluster matched, remove from contention
			if(-1 != minID)
			{
				/// remove minID element
				if(pedestrian_vision_vector.pd_vector.size())
					pedestrian_vision_vector.pd_vector.erase(pedestrian_vision_vector.pd_vector.begin()+minID);
			}
		}
		
	}
    
    for(int ii=0 ; ii<pedestrian_vision_vector.pd_vector.size(); ii++)
    {
		if(pedestrian_vision_vector.pd_vector[ii].cluster.centroid.x!=0 || pedestrian_vision_vector.pd_vector[ii].cluster.centroid.y!=0)
		{
			PED_DATA_ASSOC newPed;
			newPed.id = latest_id++;
			newPed.ped_pose = pedestrian_vision_vector.pd_vector[ii].cluster.centroid;
			cout << "Creating new pedestrian from vision with id #" << latest_id << " at x:" << newPed.ped_pose.x << " y:" << newPed.ped_pose.y << endl;
			lPedInView.push_back(newPed);
		}
	}
	
	publishPed();
	
}

void data_assoc::pedClustCallback(feature_detection::clusters cluster_vector)
{
    frame_id_ = cluster_vector.header.frame_id;
	/// loop over clusters to match with existing lPedInView
	for(int jj=0; jj< lPedInView.size(); jj++)
	{
		double minDist=10000;
		int minID=-1;
		for(int ii=0; ii < cluster_vector.clusters.size(); ii++)
		{
			double currDist = dist(lPedInView[jj].ped_pose, cluster_vector.clusters[ii].centroid);
			if( (currDist < minDist) && currDist>-1)
			{
				minDist = currDist;
				minID = ii;
			}
		}		
		
		if(minDist < NN_MATCH_THRESHOLD)
		{
			
			/// if cluster matched, remove from contention
			if(-1 != minID)
			{
				cout << " Cluster matched with ped id #" << minID << endl;
				
				lPedInView[jj].ped_pose = cluster_vector.clusters[minID].centroid;

				/// remove minID element
				if(cluster_vector.clusters.size())
					cluster_vector.clusters.erase(cluster_vector.clusters.begin()+minID);
			}
		}
		
	}
	///// Add remaining clusters as new pedestrians
	///// check with caveat .... Or just ignore new clusters
	//for(int ii=0; ii< cluster_vector.clusters.size(); ii++)
	//{
		//If satisfies some criterion  or should we ignore and 
		//let the HoG find proper pedestrians.
		
		//PED_DATA_ASSOC ped;
		//ped.id = assign some id; 
		//ped.ped_pose = cluster_vector.clusters[ii].centroid;
	//}
	
	publishPed();
}

void data_assoc::publishPed()
{
    dataAssoc_experimental::PedDataAssoc_vector lPed;
    sensor_msgs::PointCloud pc;
    
    pc.header.frame_id = frame_id_;
    pc.header.stamp = ros::Time::now();
    for(int ii=0; ii <lPedInView.size(); ii++)
    {
		dataAssoc_experimental::PedDataAssoc ped;
		ped.id = lPedInView[ii].id;
		ped.ped_pose = lPedInView[ii].ped_pose;

		lPed.ped_vector.push_back(ped);

		geometry_msgs::Point32 p;
		p = lPedInView[ii].ped_pose;
		p.z = lPedInView[ii].id;
		pc.points.push_back(p);
	}
    pedPub_.publish(lPed);
    visualizer_.publish(pc);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_assoc");

    data_assoc *data_assoc_node = new data_assoc(argc, argv);

    ros::spin();
}
