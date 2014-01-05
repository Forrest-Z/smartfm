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
//#include <dataAssoc_experimental/PedDataAssoc_vector.h>
//#include <dataAssoc_experimental/PedDataAssoc.h>
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
//#include "executer.h"
#include  "world_simulator.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "pedestrian_changelane.h"
#include "mcts.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
    double rob_pose;

    SharedPointer<BeliefWithState> currBelSt;
    int currAction;
    int currSVal; /// observed variable
    ros::Time last_update;
	bool obss_updated;
    /// some grid representation
    //frame id
};



class ped_momdp
{
public:
    vector<PED_MOMDP> lPedInView;

    ped_momdp(string model_file, string policy_file, int simLen, int simNum, bool stationary, double frequency, bool use_sim_time, ros::NodeHandle& nh,WorldSimulator *);
    
    ~ped_momdp();
    
    void updateRobotSpeed(double speed);

    void addNewPed(ped_momdp_sarsop::ped_local_frame &ped);

    bool updatePedRobPose(ped_momdp_sarsop::ped_local_frame &ped);

    void updateSteerAnglePublishSpeed(geometry_msgs::Twist speed);
	
	//void simLoop();
	void publishROSState();
	void publishBelief();
	void publishMarker(int id,vector<double> belief);

    //vector<Executer*> Executers; 
	//WorldSimulator world;
	WorldSimulator * RealWorldPt;
	MCTS*solver;

	PEDESTRIAN_CHANGELANE* RealSimulator;
	
	//void initSimulator();
	void initRealSimulator();
	void momdpInit();
	//void updatePedPoses();
	//void updateObsStates();
	//void clean_momdp_problem_sim();

	ros::Publisher window_pub;
	ros::Publisher car_pub;
	ros::Publisher pa_pub;
	ros::Publisher markers_pubs[ModelParams::N_PED_IN];

private:
	double control_freq;
	int safeAction;
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
	//SharedPointer<AlphaVectorPolicy> policy;
	//SharedPointer<AlphaVectorPolicy> qmdp_policy;
    SharedPointer<AlphaVectorPolicy> policies[5];
	SharedPointer<AlphaVectorPolicy> qmdp_policies[5];
    ROS_SimulationEngine engine;
    map<string, int> ObsStateMapping;
    map<string, int> ObsSymbolMapping;
    ros::Publisher believesPub_, cmdPub_;


    void initPedGoal();
    void initPedMOMDP(ped_momdp_sarsop::ped_local_frame& ped_local);
    void controlLoop(const ros::TimerEvent &e);

    void clean_momdp_problem();
    

    void publish_belief();

    int policy_initialize(string model_file, string policy_file, int simLen, int simNum);

    void updateBelief(int id, int safeAction);
    void updatePomcpBelief(int i, int safeAction);

    int getCurrentState(double currRobSpeed, double roby, double pedx, double pedy,int ped_id=-1);

    int getCurrObs(int id);

    int getXGrid(double x);

    int getYGrid(double y);

	//int reactivePolicy(int i);
	//int getActionMOMDP(int i);
	//int getActionQMDP(int);
	//int getActionBML(int);
};
#endif /* MOMDP_H_ */
