/*
 * pedestrian_momdp.cpp
 *
 *  Created on: Sep 15, 2011
 *      Author: golfcar
 */

#include "pedestrian_momdp_realPed.h"
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "world_simulator.h"

//WorldSimulator RealWorld;
pedestrian_momdp::pedestrian_momdp()
{
    ROS_INFO("Starting Pedestrian Avoidance ... ");

    /// Setting up subsciption
    ros::NodeHandle nh;
    speedSub_ = nh.subscribe("odom", 1, &pedestrian_momdp::speedCallback, this);
    pedSub_ = nh.subscribe("ped_local_frame_vector", 1, &pedestrian_momdp::pedPoseCallback, this); 

    ros::NodeHandle n("~");

    string  ped_id_file, policy_file, model_file;
    double frequency;
    int simLen, simNum;
    bool use_sim_time, stationary;
    n.param("pedestrian_id_file", ped_id_file, string(""));
    n.param("policy_file", policy_file, string(""));
    n.param("model_file", model_file, string(""));
    n.param("simLen", simLen, 100);
    n.param("simNum", simNum, 100);
    n.param("frequency", frequency, 2.0);
    n.param("stationary",stationary, false);
    nh.param("use_sim_time", use_sim_time, false);

    move_base_speed_=nh.subscribe("/speed_advisor_cmdvel",1, &pedestrian_momdp::moveSpeedCallback, this);
    //goalPub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
/*
	momdp = new ped_momdp(model_file, policy_file, simLen, simNum, stationary, frequency, use_sim_time, nh);
	momdp->RealWorldPt=&RealWorld;
	//momde->world=&world;
//	Executer* exec=new Executer();
//	momdp->Executers.push_back(exec);
	momdp->window_pub=nh.advertise<geometry_msgs::PolygonStamped>("/my_window",1000);
	momdp->pa_pub=nh.advertise<geometry_msgs::PoseArray>("my_poses",1000);
	momdp->car_pub=nh.advertise<geometry_msgs::PoseStamped>("car_pose",1000);
*/
	//momdp->simLoop();
	new WorldSimulator();
	cout<<"here"<<endl;
    //ros::spin();
}

pedestrian_momdp::~pedestrian_momdp()
{
    //momdp->~ped_momdp();
}

void pedestrian_momdp::speedCallback(nav_msgs::Odometry odo)
{
	//cout<<"speed callback!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    //momdp->updateRobotSpeed(odo.twist.twist.linear.x);
	//RealWorld.UpdateVelReal(odo.twist.twist.linear.x);
}

void pedestrian_momdp::moveSpeedCallback(geometry_msgs::Twist speed)
{

    momdp->updateSteerAnglePublishSpeed(speed);

}

void pedestrian_momdp::pedPoseCallback(ped_momdp_sarsop::ped_local_frame_vector lPedLocal)
{
	if(lPedLocal.ped_local.size()==0) return;
	ped_momdp_sarsop::ped_local_frame ped=lPedLocal.ped_local[0];
	Car world_car;
	world_car.w=ped.rob_pose.x*ModelParams::rln;
	world_car.h=ped.rob_pose.y*ModelParams::rln;
	//RealWorld.UpdateRobPoseReal(world_car);
	//update the ped poses
    for(int ii=0; ii< lPedLocal.ped_local.size(); ii++)
    {
		Pedestrian world_ped;
		ped_momdp_sarsop::ped_local_frame ped=lPedLocal.ped_local[ii];
		world_ped.id=ped.ped_id;
		world_ped.w = ped.ped_pose.x*ModelParams::rln;
		world_ped.w = ped.ped_pose.y*ModelParams::rln;

			
        /// search for proper pedestrian to update
        //bool foundPed = momdp->updatePedRobPose(lPedLocal.ped_local[ii]);;
		//RealWorld.UpdatePedPoseReal(world_ped);

        //if(!foundPed)
        //{
            ///if ped_id does not match the old one create a new pomdp problem.
            //ROS_INFO(" Creating  a new pedestrian problem #%d", lPedLocal.ped_local[ii].ped_id);
            //momdp->addNewPed(lPedLocal.ped_local[ii]);
			//ROS_INFO("Create a new pedestrian!", lPedLocal.ped_local[ii].ped_id);
			//world.addNewPed(lPedLocal.ped_local[ii]);                                                  }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mdp");


	
    srand(unsigned(time(0)));
    pedestrian_momdp mdp_node;


}
