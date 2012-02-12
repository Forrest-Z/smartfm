/*
 * data_assoc.h
 *
 *  Created on: Sep 15, 2011
 *      Author: golfcar
 */

#ifndef DATA_ASSOC_H_
#define DATA_ASSOC_H_
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include <sensing_on_road/pedestrian_vision_batch.h>
#include <feature_detection/clusters.h>
#include <geometry_msgs/Point32.h>
#include <dataAssoc_experimental/PedDataAssoc.h>
#include <dataAssoc_experimental/PedDataAssoc_vector.h>
using namespace std;

#define NN_MATCH_THRESHOLD 0.5

struct PED_DATA_ASSOC
{
	int id;        
	geometry_msgs::Point32 ped_pose; /// updated from the centroid                 
};

vector<PED_DATA_ASSOC> lPedInView;

class data_assoc 
{
public:
    data_assoc(int argc, char** argv);
    //data_assoc();
    ~data_assoc();
    
	void pedClustCallback(feature_detection::clusters cluster_vector); 
    void pedVisionCallback(sensing_on_road::pedestrian_vision_batch pedestrian_vision_vector);
    
    void publishPed();
    
    

    ros::Subscriber pedVisionSub_;
    ros::Subscriber pedClustSub_;
    ros::Publisher pedPub_;
    ros::Publisher visualizer_;
    string frame_id_;
    bool use_sim_time_;
    //double robotx_, roboty_, robotspeedx_;//pedx_, pedy_;
    
    //double robotspeedx_;
    //int simLen, simNum;
    //string  ped_id_file, policy_file, model_file;
    //void parse_simul_config( fstream& configfile);
    
    //int policy_initialize();
    //void pedInitPose();
    
    //void initPedMOMDP(ped_momdp_sarsop::ped_local_frame ped_local);
    

    //ofstream* foutStream;

	//struct POSE
	//{
		//double x;
		//double y;
		//double yaw;
	//};
	
	int latest_id;
    

    //ros::Timer timer_;
};

#endif /* DATA_ASSOC_H_ */
