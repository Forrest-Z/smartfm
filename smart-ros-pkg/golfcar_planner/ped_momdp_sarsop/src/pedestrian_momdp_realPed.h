/*
 * pedestrian_momdp.h
 *
 *  Created on: Sep 15, 2011
 *      Author: golfcar
 */

#ifndef PEDESTRIAN_MOMDP_H_
#define PEDESTRIAN_MOMDP_H_
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include "MOMDP.h"
#include "ParserSelector.h"
#include "AlphaVectorPolicy.h"
#include "PSG_SimulationEngine.h"
#include "GlobalResource.h"
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ped_momdp_sarsop/peds_believes.h>

//bool lane=true;
//bool cross=!lane;


bool myDebug=false; // moves rob in discrete steps
bool movePed = false;
bool moveRob = true;
bool mostLikelyAlgo; /// read from config.txt

struct GOAL
{
	int x, y;
};
vector<GOAL> lPedGoal;

//int currRobY;
//int currRobVel;


#define NUM_PED 1

/// for visualization
#define NUM_GOAL 4

struct player_point_2d_t
{
	double px;
	double py;
};

player_point_2d_t pedPt[NUM_PED]; /// current pos
player_point_2d_t goalPt[NUM_GOAL]; /// all goal points


/// simulation run
int pedGoal_x[NUM_PED];// = 9;
int pedGoal_y[NUM_PED];// = 7;

//double dY=64.0/Y_SIZE;
//double dX=16.0/X_SIZE;
//double Y_OFFSET=32;
//double X_OFFSET=8;


int X_SIZE=4;
int Y_SIZE=10;

//double ROS_Y=;
//double ROS_X=;

double dY= 2;//ROS_Y/Y_SIZE; /// step size in Y
double dX= 2;//ROS_X/X_SIZE; /// step size in X
//double Y_OFFSET=PSG_Y/2.0;  /// getting zero to bot-left
//double X_OFFSET=PSG_X/2.0;

double X_OFFSET = 3;
double Y_OFFSET = 0;

double dSpeed=dY; /// Acc
double currRobSpeed=0;
#define MAX_ROB_SPEED (2*dY+0.1)

double SLEEP=dY/dSpeed;
bool obs_flag=false;
bool obs_first=true;
bool robot_pose=false;
/// Generate Stats
ofstream statfile;
int num_steps=0;
//int YXSwitch; /// place where trajectory changes from vertical to Horizontal
//int XYSwitch; /// from horizontal to vertical
//double closest_dist=0;
class pedestrian_momdp {
public:
	pedestrian_momdp(int argc, char** argv);
	virtual ~pedestrian_momdp();
	void robotPoseCallback(geometry_msgs::PoseWithCovarianceStamped odo);
	void speedCallback(nav_msgs::Odometry odo);
	void pedPoseCallback(sensing_on_road::pedestrian_laser_batch laser_batch);
	void controlLoop(const ros::TimerEvent &e);
	void publish_belief();
	int getCurrentState(int id);
	int getCurrObs(int id);
	int getXGrid(double x);
	int getYGrid(double y);
	void initPedGoal();

	ros::Subscriber robotSub_, speedSub_, pedSub_, scanSub_;
	ros::Publisher cmdPub_, believesPub_;
	double robotx_, roboty_, robotspeedx_;//pedx_, pedy_;
	int simLen, simNum;
	string  ped_id_file, policy_file, model_file;
	void parse_simul_config( fstream& configfile);
	int pomdp_initialize();
	void pedInitPose();

	ofstream* foutStream;
	
	struct PED
	{
		int id;
		double pedx_;
		double pedy_;
	};
	vector<PED> lPedInView;
	
	geometry_msgs::Twist cmd;
	//int currAction[];
	SolverParams* p;
	SharedPointer<MOMDP> problem;
	SharedPointer<AlphaVectorPolicy> policy;
	PSG_SimulationEngine engine;

    int currSVal[NUM_PED];
    vector< SharedPointer<BeliefWithState> > lcurrBelSt;
    int currAction[NUM_PED];

    double mult;
    double gamma;

    map<string, int> ObsStateMapping;
    map<string, int> ObsSymbolMapping;

    ros::Timer timer_;
};

#endif /* PEDESTRIAN_MOMDP_H_ */
