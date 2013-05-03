/*
 * Veicle_momdp_realveh.cpp
 *
 *  Created on: April 30, 2013
 *      Author: golfcar
 */

#include "Vehicle_momdp_realPed_realveh.h"


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

    momdp = new ped_momdp(model_file, policy_file, simLen, simNum, stationary, frequency, use_sim_time, nh);

    ros::spin();
}

pedestrian_momdp::~pedestrian_momdp()
{
    momdp->~ped_momdp();
}

void pedestrian_momdp::speedCallback(nav_msgs::Odometry odo)
{
    momdp->updateRobotSpeed(odo.twist.twist.linear.x);
}

void pedestrian_momdp::moveSpeedCallback(geometry_msgs::Twist speed)
{
    momdp->updateSteerAnglePublishSpeed(speed);

}

void pedestrian_momdp::pedPoseCallback(ped_momdp_sarsop::ped_local_frame_vector lPedLocal)
{
    for(int ii=0; ii< lPedLocal.ped_local.size(); ii++)
    {
        /// search for proper pedestrian to update
        bool foundPed = momdp->updatePedRobPose(lPedLocal.ped_local[ii]);;

        if(!foundPed)
        {
            ///if ped_id does not match the old one create a new pomdp problem.
            ROS_INFO(" Creating  a new pedestrian problem #%d", lPedLocal.ped_local[ii].ped_id);
            momdp->addNewPed(lPedLocal.ped_local[ii]);
        }
    }


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mdp");
    pedestrian_momdp mdp_node;


}
