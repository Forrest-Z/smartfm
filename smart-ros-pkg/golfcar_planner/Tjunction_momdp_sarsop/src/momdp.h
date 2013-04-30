/*
 * momdp.h
 *
 *  Created on: Mar 4, 2012
 *      Author: golfcar
 */

#ifndef MOMDP_H_
#define MOMDP_H_

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include <dataAssoc_experimental/PedDataAssoc_vector.h>
#include <dataAssoc_experimental/PedDataAssoc.h>
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
#include <pnc_msgs/speed_contribute.h>



 #include <std_msgs/Float32.h>

using namespace std;

struct GOAL
{
    int x, y;
};

struct POSE
{
    double x;
    double y;
    double yaw;
};

struct PED_MOMDP
{
    int id;
    POSE ped_pose;
 //V   double rob_pose;
    POSE rob_pose; //V

    SharedPointer<BeliefWithState> currBelSt;
    int currAction;
    int currSVal; /// observed variable
    ros::Time last_update;
    /// some grid representation
    //frame id
};



class ped_momdp
{
public:
    vector<PED_MOMDP> lPedInView;

    ped_momdp(string model_file, string policy_file, int simLen, int simNum, bool stationary, double frequency, bool use_sim_time, ros::NodeHandle& nh);
    
    ~ped_momdp();
    
    void updateRobotSpeed(double speed);

    void addNewPed(ped_momdp_sarsop::ped_local_frame &ped);

    bool updatePedRobPose(ped_momdp_sarsop::ped_local_frame &ped);

    void updateSteerAnglePublishSpeed(geometry_msgs::Twist speed);

    int ped_str_pub;

    geometry_msgs::Twist volkan;// This is not the correct way to see the robot state. Just for 
    



private:
    int X_SIZE, Y_SIZE;
    double dX, dY;
    double momdp_problem_timeout;
    bool robot_pose_available;
    double robotx_, roboty_, robotspeedx_, momdp_speed_;
    bool use_sim_time_, stationary_;
    ros::Timer timer_;
    vector<GOAL> lPedGoal;
    SolverParams* solver_param;
    SharedPointer<MOMDP> problem;
    SharedPointer<AlphaVectorPolicy> policy;
    ROS_SimulationEngine engine;
    map<string, int> ObsStateMapping;
    map<string, int> ObsSymbolMapping;
    ros::Publisher believesPub_, cmdPub_, RobState_pub,	AnalysisCrash_pub, AnalysisTime_pub;

    std_msgs::Float32 analysisvar;
    double total_belief_value;




    void initPedMOMDP(ped_momdp_sarsop::ped_local_frame& ped_local);
    void controlLoopMOMDP(const ros::TimerEvent &e);//Controller Using MOMDP
    void controlLoopConstantGap0(const ros::TimerEvent &e);//Controller Using Constant Gap Approach(Gap-0)
    void controlLoopConstantGap1(const ros::TimerEvent &e);//Controller Using Constant Gap Approach(Gap-1)
    void controlLoopConstantGap2(const ros::TimerEvent &e);//Controller Using Constant Gap Approach(Gap-2)
    void controlLoopConstantGap3(const ros::TimerEvent &e);//Controller Using Constant Gap Approach(Gap-3)


    void clean_momdp_problem();
    

    void publish_belief(); //No Need for this function, only for analysis!

    int policy_initialize(string model_file, string policy_file, int simLen, int simNum);

    void updateBelief(int id, int safeAction);

    int getCurrentState(double robx, double roby, double pedx, double pedy);

    int getCurrObs(int id);

    int getXGrid(double x);

    int getYGrid(double y);
};
#endif /* MOMDP_H_ */
