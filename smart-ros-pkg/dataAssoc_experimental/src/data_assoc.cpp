
#include "data_assoc.h"

using namespace std;




data_assoc::data_assoc(int argc, char** argv) 
{
	ROS_INFO("Starting Pedestrian Avoidance ... ");
	
	/// Setting up subsciption 
    ros::NodeHandle nh;
    
    pedClustSub_.subscribe(nh, "ped_laser_cluster", 10);
    pedVisionSub_.subscribe(nh, "ped_vision", 10);
    /// TBP : how to add multiple subscription to same call back ????
    
    ros::NodeHandle n("~");
    
    n.param("global_frame", global_frame_, string("odom"));
    n.param("time_out", time_out_, 3.0);
    /// Setting up publishing
    pedPub_ = nh.advertise<feature_detection::clusters>("ped_data_assoc",1); /// topic name
    visualizer_ = nh.advertise<sensor_msgs::PointCloud>("ped_data_assoc_visual",1);
	latest_id=0;

	listener_ = new tf::TransformListener(ros::Duration(10));

	/// Setting up callback with transform cache
	laser_tf_filter_ = new tf::MessageFilter<feature_detection::clusters>(pedClustSub_, *listener_, global_frame_, 10);
	laser_tf_filter_->registerCallback(boost::bind(&data_assoc::pedClustCallback, this, _1));

	vision_tf_filter_ = new tf::MessageFilter<sensing_on_road::pedestrian_vision_batch>(pedVisionSub_, *listener_, global_frame_, 10);
	vision_tf_filter_ ->registerCallback(boost::bind(&data_assoc::pedVisionCallback, this, _1));

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

bool data_assoc::transformPointToGlobal(std_msgs::Header header, geometry_msgs::Point32 input_point, geometry_msgs::Point32& output_point)
{
    //why there is geometry_msgs::Point32 and geometry_msgs??
    try{
    geometry_msgs::PointStamped global_point, local_point;
    local_point.header = header;
    local_point.point.x = input_point.x;
    local_point.point.y = input_point.y;
    local_point.point.z = input_point.z;
    listener_->transformPoint(global_frame_, local_point, global_point);
    geometry_msgs::Point32 p32;
    p32.x = global_point.point.x; p32.y = global_point.point.y; p32.z = global_point.point.z;
    output_point = p32;
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform point: %s", ex.what());
    }

    return true;
}
void data_assoc::pedVisionCallback(sensing_on_road::pedestrian_vision_batchConstPtr pedestrian_vision_vector)
{
    std::vector<sensing_on_road::pedestrian_vision> pd_vector = pedestrian_vision_vector->pd_vector;
    ROS_DEBUG(" Entering vision call back with lPedInView %d and ped vector size of %d", lPedInView.clusters.size(), pd_vector.size());

	//pedestrian_vision_vector.pd_vector[].cluster.centroid;
	/// loop over clusters to match with existing lPedInView
	for(int jj=0; jj< lPedInView.clusters.size(); jj++)
	{
		double minDist=10000;
		int minID=-1;
		for(int ii=0; ii < pd_vector.size(); ii++)
		{
		    geometry_msgs::Point32 global_point;
		    bool transformed = transformPointToGlobal(pedestrian_vision_vector->header, pd_vector[ii].cluster.centroid, global_point);
		    if(!transformed) return;
			double currDist = dist(lPedInView.clusters[jj].centroid, global_point);
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
			    ROS_DEBUG("From Vision: Cluster %d matched with dist %lf with decision flag %d", lPedInView.clusters[jj].id, minDist, pd_vector[minID].decision_flag);
				/// remove minID element
				if(pd_vector.size())
					pd_vector.erase(pd_vector.begin()+minID);
			}
		}
		
	}

    for(int ii=0 ; ii<pd_vector.size(); ii++)
    {

		if(pd_vector[ii].decision_flag)//(pedestrian_vision_vector.pd_vector[ii].cluster.centroid.x!=0 || pedestrian_vision_vector.pd_vector[ii].cluster.centroid.y!=0)
		{
			feature_detection::cluster newPed;
			newPed.id = latest_id++;
			bool transformed = transformPointToGlobal(pedestrian_vision_vector->header, pd_vector[ii].cluster.centroid, newPed.centroid);
			if(!transformed) return;
			ROS_INFO_STREAM( "Creating new pedestrian from vision with id #" << latest_id << " at x:" << newPed.centroid.x << " y:" << newPed.centroid.y);
			lPedInView.clusters.push_back(newPed);
		}
	}
	
	publishPed();
	
}

void data_assoc::pedClustCallback(feature_detection::clustersConstPtr cluster_vector)
{
    frame_id_ = cluster_vector->header.frame_id;
    ROS_DEBUG_STREAM( " Entering pedestrian call back with lPedInView " << lPedInView.clusters.size() );
	/// loop over clusters to match with existing lPedInView
    std::vector<feature_detection::cluster> clusters = cluster_vector->clusters;
    feature_detection::clusters clusters_visualize;
    clusters_visualize.header = cluster_vector->header;
    lPedInView.header = cluster_vector->header;
    lPedInView.header.frame_id = global_frame_;
	for(int jj=0; jj< lPedInView.clusters.size(); jj++)
	{
		double minDist=10000;
		int minID=-1;
		for(int ii=0; ii < clusters.size(); ii++)
		{
		    geometry_msgs::Point32 global_point;
		    bool transformed = transformPointToGlobal(cluster_vector->header, clusters[ii].centroid, global_point);
		    if(!transformed) return;
			double currDist = dist(lPedInView.clusters[jj].centroid, global_point);
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
				ROS_DEBUG_STREAM(" Cluster matched with ped id #" << lPedInView.clusters[jj].id );
				geometry_msgs::Point32 global_point;
				bool transformed = transformPointToGlobal(cluster_vector->header, clusters[minID].centroid,global_point);
				if(!transformed) return;
				//we are update everything except the id;
				int id = lPedInView.clusters[jj].id;
				lPedInView.clusters[jj] = clusters[minID];
				lPedInView.clusters[jj].id = id;
				lPedInView.clusters[jj].centroid = global_point;
				lPedInView.clusters[jj].last_update = ros::Time::now();
				/// remove minID element
				if(clusters.size())
					clusters.erase(clusters.begin()+minID);
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
	cleanUp();
	publishPed();
}

void data_assoc::cleanUp()
{
    for(int jj=0; jj< lPedInView.clusters.size(); )
    {
        ros::Duration unseen_time = ros::Time::now() - lPedInView.clusters[jj].last_update;
        ROS_DEBUG("ped with ID %d unseen time = %lf", lPedInView.clusters[jj].id, unseen_time.toSec());
        if(unseen_time.toSec()>time_out_)
        {
            ROS_INFO("Erase ped with ID %d due to time out", lPedInView.clusters[jj].id);
            lPedInView.clusters.erase(lPedInView.clusters.begin()+jj);

        }
        else
            jj++;

    }
}

void data_assoc::publishPed()
{
    dataAssoc_experimental::PedDataAssoc_vector lPed;
    sensor_msgs::PointCloud pc;
    
    pc.header.frame_id = global_frame_;
    pc.header.stamp = ros::Time::now();
    for(int ii=0; ii <lPedInView.clusters.size(); ii++)
    {
		geometry_msgs::Point32 p;
		p = lPedInView.clusters[ii].centroid;
		p.z = lPedInView.clusters[ii].id;
		pc.points.push_back(p);
	}
    pedPub_.publish(lPedInView);
    visualizer_.publish(pc);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_assoc");

    data_assoc *data_assoc_node = new data_assoc(argc, argv);

    ros::spin();
}
