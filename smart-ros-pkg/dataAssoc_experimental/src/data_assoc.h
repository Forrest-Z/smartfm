/*
 * data_assoc.h
 *
 *  Created on: Sep 15, 2011
 *      Author: golfcar
 */

#ifndef DATA_ASSOC_H_
#define DATA_ASSOC_H_
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include "MOMDP.h"
#include "ParserSelector.h"
#include "AlphaVectorPolicy.h"
#include "ROS_SimulationEngine.h"
#include "GlobalResource.h"
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ped_momdp_sarsop/peds_believes.h>
#include <ped_momdp_sarsop/ped_local_frame.h>
#include <ped_momdp_sarsop/ped_local_frame_vector.h>
#include <pnc_msgs/move_status.h>

class data_assoc 
{
public:
    data_assoc(int argc, char** argv);
    ~data_assoc();
    
    void pedClustCallback(ped_momdp_sarsop::ped_local_frame_vector ped_local_vector, perception_experimental::clusters cluster_vector );   
    
    void publishPed();
    
    

    ros::Subscriber pedSub_;
    ros::Publisher pedPub_;
    
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
	
    struct PED_DATA_ASSOC
    {
        int id;        
		geometry_msgs::Point32 ped_pose; /// updated from the centroid                 
    };
    vector<PED_DATA_ASSOC> lPedInView;

    ros::Timer timer_;
};

#endif /* DATA_ASSOC_H_ */
