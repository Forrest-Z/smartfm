/**
 * @file SimulationDriver.cc
 * @brief This is the simulation driver which will call the necessary functions and make
 * the correct calls as and when required
 * @author Amit Jain
 * @date 2007-04-10
 */


#include <libplayerc++/playerc++.h>
#include <iostream>


#include "PSG_SimulationEngine.h"
#include "GlobalResource.h"
#include "SimulationRewardCollector.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ctime>

#include "CPTimer.h"

#ifdef _MSC_VER
#else
//for timing
#include <sys/param.h>
#include <sys/types.h>
#include <sys/times.h>
//end for timing
#endif

#include "MOMDP.h"
#include "ParserSelector.h"
#include "AlphaVectorPolicy.h"

using namespace std;
using namespace PlayerCc;
using namespace momdp;


/// lane figure
double PSG_Y=60.0;
double PSG_X=14.0; 
#define X_SIZE 4
#define Y_SIZE 10
bool lane=true;

/// intersection
//double PSG_X=64.0;   
//double PSG_Y=64.0;
//#define X_SIZE 15
//#define Y_SIZE 11
bool cross=!lane;


bool myDebug=false; // moves rob in discrete steps
bool movePed = true;
bool moveRob = true;	
bool mostLikelyAlgo; /// read from config.txt

struct GOAL
{
	int x, y;
};
vector<GOAL> lPedGoal;

//int currRobY;
//int currRobVel;

#include "args.h"

#define NUM_PED 1
SimulationProxy* sp;
Graphics2dProxy* gp[NUM_PED];

PlayerClient * ped[NUM_PED];

Position2dProxy* pp;
Position2dProxy* ped_pp[NUM_PED];
LaserProxy* lp;

/// for visualization
#define NUM_GOAL 4
player_point_2d_t pedPt[NUM_PED]; /// current pos
player_point_2d_t goalPt[NUM_GOAL]; /// all goal points


/// simulation run
int pedGoal_x[NUM_PED];// = 9;
int pedGoal_y[NUM_PED];// = 7;

//double dY=64.0/Y_SIZE;
//double dX=16.0/X_SIZE;
//double Y_OFFSET=32;
//double X_OFFSET=8;




double dY= PSG_Y/Y_SIZE; /// step size in Y
double dX= PSG_X/X_SIZE; /// step size in X
double Y_OFFSET=PSG_Y/2.0;  /// getting zero to bot-left
double X_OFFSET=PSG_X/2.0;

double dSpeed=dY; /// Acc 
double currRobSpeed=0;
#define MAX_ROB_SPEED (2*dY+0.1)

double SLEEP=dY/dSpeed;

map<string, int> ObsStateMapping;
map<string, int> ObsSymbolMapping;

/// Generate Stats
ofstream statfile;
int num_steps=0;
int YXSwitch; /// place where trajectory changes from vertical to Horizontal
int XYSwitch; /// from horizontal to vertical
//double closest_dist=0;


/////////////////////////////////////////
int getXGrid(double x)
{
	int px = (int) (x+X_OFFSET)/dX ;
	return px;
}

int getYGrid(double y)
{
	int py = (int) (y+Y_OFFSET)/dY ;
	return py;
}

double getXPos(int x)
{
	double dx = ( x) * dX -X_OFFSET +0.5*dX;
	
	//cout << " get X " << x << " dx " << dx << " dX " << dX  << endl;
	
	return dx;
}

double getYPos(int y)
{	
	double dy = (y) * dY -Y_OFFSET +0.5*dY;
	
	//cout << " get Y " << y << " dy " << dy << " dY " << dY << endl;
	//cout << " PSG_Y " << PSG_Y << " Y_SIZE " << Y_SIZE << " == " << PSG_Y/Y_SIZE << endl; 
	return dy;
	
	
}


void executeActions(string action)
{
	cout << " Execute Actions ...  " << endl;
	/// execute the action
	
	double rob_x,rob_y,rob_yaw;		
	sp->GetPose2d("rob", rob_x, rob_y, rob_yaw);
	
	//double ped_x,ped_y,ped_yaw;	
	//sp->GetPose2d("ped", ped_x, ped_y, ped_yaw);
	
	/////-------------
	
	//int px = getXGrid(ped_x);
	
	//int py = getYGrid(ped_y);
	int ry = getYGrid(rob_y);
	
	
	//cout << " Before execute px " << ped_x << ":" << px << " py " << ped_y <<":" << py << " ry " << rob_y << ":" << ry << " robvel "  <<   currRobSpeed << endl;
	
	
	//if(py>Y_SIZE)
		//exit(1);

	if(ry>Y_SIZE-2)
	{
		cout << "Robot reached goal " << endl;
		exit(1);	
	}
		
	//if(px>X_SIZE)
		//exit(1);	
		
	///-------------
	
	if(strcmp(action.c_str(),"aAcc")==0)
	{
		if(currRobSpeed < MAX_ROB_SPEED)
			currRobSpeed += dSpeed;
		
		if(currRobSpeed > MAX_ROB_SPEED)
			currRobSpeed = MAX_ROB_SPEED;
		
		if(myDebug)
			sp->SetPose2d("rob", rob_x, rob_y+currRobSpeed, 1.57);
		else
			pp->SetSpeed(currRobSpeed,0);
		
	}
	else if(strcmp(action.c_str(),"aDec")==0)
	{
		if(currRobSpeed >0)
			currRobSpeed -= dSpeed;
			
		if(currRobSpeed<0)
			currRobSpeed =0;
		
		if(myDebug)
			sp->SetPose2d("rob", rob_x, rob_y+currRobSpeed, 1.57);
		else
			pp->SetSpeed(currRobSpeed,0);
		
	}
	else if(strcmp(action.c_str(),"aCru")==0)
	{
		/// cruising

		if(myDebug)
			sp->SetPose2d("rob", rob_x, rob_y+currRobSpeed, 1.57);
		else
			pp->SetSpeed(currRobSpeed,0);
		
	}
	else
	{
		cout << "Error executing actions .. " << endl;
		pp->SetSpeed(0,0);
		currRobSpeed=0;
	}
	
	sleep(SLEEP);
	
	//{
		//double rob_x1,rob_y1,rob_yaw1;		
		//sp->GetPose2d("rob", rob_x1, rob_y1, rob_yaw1);
	
		//double ped_x1,ped_y1,ped_yaw1;	
		//sp->GetPose2d("ped", ped_x1, ped_y1, ped_yaw1);
		
		/////-------------
		
		//int px1 = getXGrid(ped_x1);
		
		//int py1 = getYGrid(ped_y1);
		//int ry1 = getYGrid(rob_y1);	
		//cout << " After execute px " << ped_x1 << ":" << px1 << " py " << ped_y1 <<":" << py1 << " ry " << rob_y1 << ":" << ry1 << " robVel " << currRobSpeed << endl;
	//}
	
	return;
}

bool flag=true;
void MovePedGoTo(int id, int pedG_x, int pedG_y)
{
	double ped_goal_x = getXPos(pedG_x);
	double ped_goal_y = getYPos(pedG_y);
	
	//ped_pp[id]->SetSpeed(4,0);
	//player_pose2d_t pos; 
	//pos.px = ped_goal_x;
	//pos.py = ped_goal_y;
	//pos.pa = 0;
	
	
	char nameid[10];
	sprintf(nameid,"ped%d",id);
	double ped_x,ped_y,ped_yaw;	
	sp->GetPose2d(nameid, ped_x, ped_y, ped_yaw);

	double dx=0;
	double dy=0;
	
	dx = ped_goal_x - ped_x;	
	dy = ped_goal_y - ped_y;
	
	cout << " ped goal #" << id << " " << ped_goal_x << ":" << ped_goal_y << " dx:" << dx << ", dy:" << dy << endl; 
	//double maxPedSpeed=dX;
	double kp=0.75;
	if(fabs(dx) > 1)
		dx=dX*dx/fabs(dx) * kp;
	else
		dx=0;
	
	if(fabs(dy) > 1)
		dy=dY*dy/fabs(dy) *kp;
	else
		dy=0;
	
	//if(flag)
	//{
		//ped_pp[id]->SetSpeed(dx, 0,0);
		//flag=false;
	//}
	//else
	//{
		//ped_pp[id]->SetSpeed(0, dy,0);
		//flag=true;		
	//}
	
	ped_pp[id]->SetSpeed(dx, dy,0);
	cout << "moving ped " << dx << ", " << dy << endl;
	//sleep(SLEEP);
	
	//player_pose2d_t vel;
	//vel.px = 2;
	//vel.py = 2;
	//vel.pa = 0;
	
	
	
	//GoTo (player_pose2d_t pos, player_pose2d_t vel)
	//ped_pp[id]->GoTo(pos, vel);
	
	return;
}



void MovePed(int id)
{
	cout << "moving ped #" << id;// << endl; 
	char nameid[10];
	sprintf(nameid,"ped%d",id);
	
	double ped_x,ped_y,ped_yaw;	
	sp->GetPose2d(nameid, ped_x, ped_y, ped_yaw);
	
	//flag=false; /// horizontal
	if( getYGrid(ped_y) < YXSwitch || getXGrid(ped_x)> XYSwitch)
		flag=true; /// vertical
	else 
		flag = false; /// horizontal
		
	if(flag)
	{
		cout << " vertically " << endl;
		/// move vertical	
		double ped_y_ = ped_y+dY;
		//double ped_y_ = ped_y-dY;
		
		if( getYGrid(ped_y_) < Y_SIZE+1)
		//if( getYGrid(ped_y_) >-1)// Y_SIZE)
		{
			sp->SetPose2d(nameid, ped_x, ped_y_, ped_yaw);
			
			///// rotate
			//sp->SetPose2d("ped", ped_x, ped_y, 1.57);
			//sleep(SLEEP);
			//ped_pp->SetSpeed(dSpeed*0.5,0);
			
			flag = false;
		}
	}
	else
	{cout << " horizontally " << endl;
		/// move horizontally
		//double ped_x_ = ped_x+0.5*dX;
		double ped_x_ = ped_x+dX;
		if( getXGrid(ped_x_) < X_SIZE)
		{
			sp->SetPose2d(nameid, ped_x_, ped_y, ped_yaw);			
			
			///// rotate
			//sp->SetPose2d("ped", ped_x, ped_y, 0);
			//sleep(SLEEP);
			//ped_pp->SetSpeed(dSpeed*0.5,0);
			
			flag = true;		
		}
	}
	
	/// maintain the pedestrian in the field.
	if(ped_x>X_SIZE)
	{
		//cout << "Robot reached goal " << endl;		
		ped_pp[id]->SetSpeed(0,0);		
	}
	if(ped_y>Y_SIZE-1)
	{
		//cout << "Robot reached goal " << endl;		
		ped_pp[id]->SetSpeed(0,0);		
	}
	//sleep(SLEEP);
}


void MovePed_step(int id)
{
	cout << "moving ped #" << id << endl; 
	char nameid[10];
	sprintf(nameid,"ped%d",id);
	
	double ped_x,ped_y,ped_yaw;	
	sp->GetPose2d(nameid, ped_x, ped_y, ped_yaw);
	
	////flag=false; /// horizontal
	//if( getYGrid(ped_y) < 7)
		//flag=true; /// vertical
	//else 
		//flag = false;
		
	if(flag)
	{
		/// move vertical	
		
		//double ped_y_ = ped_y-dY;
		
		//if( getYGrid(ped_y_) < Y_SIZE)
		//if( getYGrid(ped_y_) >-1)// Y_SIZE)
		
		double dy = pedGoal_y[id] - getYGrid(ped_y);
		double ped_y_;
		if(dy)		
		 ped_y_ = ped_y+dY*dy/fabs(dy);
		else
			ped_y_ = ped_y;
		
		if( (fabs(dy)>1) && getYGrid(ped_y_) < Y_SIZE)
		{
			sp->SetPose2d(nameid, ped_x, ped_y_, ped_yaw);
			
			///// rotate
			//sp->SetPose2d("ped", ped_x, ped_y, 1.57);
			//sleep(SLEEP);
			//ped_pp->SetSpeed(dSpeed*0.5,0);
						
		}
		flag = false;
	}
	else
	{
		/// move horizontally
		//double ped_x_ = ped_x+0.5*dX;
		
		//if( getXGrid(ped_x_) < X_SIZE)
		
		double dx = pedGoal_x[id] - getXGrid(ped_x);
		double ped_x_;
		if(dx)
		 ped_x_= ped_x+dX*dx/fabs(dx);
		else
		  ped_x_ = ped_x;
		if( (fabs(dx)>1) && getXGrid(ped_x_) < X_SIZE)
		{
			sp->SetPose2d(nameid, ped_x_, ped_y, ped_yaw);			
			
			///// rotate
			//sp->SetPose2d("ped", ped_x, ped_y, 0);
			//sleep(SLEEP);
			//ped_pp->SetSpeed(dSpeed*0.5,0);
			
			
		}
		flag = true;		
	}
	
	/// maintain the pedestrian in the field.
	if(ped_x>X_SIZE)
	{
		//cout << "Robot reached goal " << endl;		
		ped_pp[id]->SetSpeed(0,0);		
	}
	if(ped_y>Y_SIZE-1)
	{
		//cout << "Robot reached goal " << endl;		
		ped_pp[id]->SetSpeed(0,0);		
	}
	//sleep(SLEEP);
}


int getCurrentState()
{
	
	int StateVal;
	
	/// Poll sensors 
	double rob_x,rob_y,rob_yaw;
	double ped_x,ped_y,ped_yaw;	
	
	sp->GetPose2d("rob", rob_x, rob_y, rob_yaw);
	sp->GetPose2d("ped", ped_x, ped_y, ped_yaw);
	
	cout << " rob " << rob_x << "," << rob_y << "," << rob_yaw << ", vel " << currRobSpeed  << endl;
	cout << " ped " << ped_x << "," << ped_y << "," << ped_yaw << endl;

//ped sx00y02
//rob sR01
//rvel sV0
// ---
//State : 5098
//PedPose_0 : sx03y16
//RobPose_0 : str
//RobVel_0 : sV2


	/// Bin continuous values to get discrete values
	
	int px = getXGrid(ped_x);
	int py = getYGrid(ped_y);
	char ped_str[10];
	sprintf(ped_str,"sx%02dy%02d",px,py);
	//if(myDebug)
	//sprintf(ped_str,"sx%02dy%02d",1,10);
	

	int ry = getYGrid(rob_y);
	char rob_str[10];
	sprintf(rob_str,"sR%02d",ry);
	
	//if(myDebug)
	//sprintf(rob_str,"sR%02d",currRobY);
	
	
	if(ry>Y_SIZE-2)
	{
		currRobSpeed=0;
		pp->SetSpeed(0,0);
		cout << "Robot Reached goal .. " << endl;
		exit(1);
	}
		
		
	
	double rvel_double =  currRobSpeed/dSpeed;
	int rvel= (int) rvel_double;
	//if(rvel_double < 0.0001 )
		//rvel = (int) rvel_double + 1; /// the int takes floor value
	//cout << "rvel_d " << rvel_double << " rvel_int " << rvel << endl; 
	
	//cout << "psg velocity " << pp->GetYSpeed() << endl;
		
	char rob_vel_str[10];
	sprintf(rob_vel_str,"sV%d",rvel);
	//cout << " rvel : " << rob_vel_str << endl;
	//if(myDebug)
	//sprintf(rob_vel_str,"sV%d",currRobVel);
	
	string state_str;
	state_str.append(ped_str);
	state_str.append(rob_str);
	state_str.append(rob_vel_str);

	/// Lookup State id
	StateVal = ObsStateMapping[state_str];

    cout << " Curr state " << state_str << " id " << StateVal << endl;	
	
	
	
	
	return StateVal;
}


int getCurrentState(int id)
{
	char nameid[10];
	sprintf(nameid,"ped%d",id);
	
	int StateVal;
	
	/// Poll sensors 
	double rob_x,rob_y,rob_yaw;
	double ped_x,ped_y,ped_yaw;	
	
	sp->GetPose2d("rob", rob_x, rob_y, rob_yaw);
	sp->GetPose2d(nameid, ped_x, ped_y, ped_yaw);
	
	pedPt[id].px = ped_x;
	pedPt[id].py = ped_y;
	
	cout << " rob " << rob_x << "," << rob_y << "," << rob_yaw << ", vel " << currRobSpeed  << endl;
	cout << " " << nameid << " " << ped_x << "," << ped_y << "," << ped_yaw << endl;

//ped sx00y02
//rob sR01
//rvel sV0
// ---
//State : 5098
//PedPose_0 : sx03y16
//RobPose_0 : str
//RobVel_0 : sV2


	/// Bin continuous values to get discrete values
	
	int px = getXGrid(ped_x);
	int py = getYGrid(ped_y);
	char ped_str[10];
	sprintf(ped_str,"sx%02dy%02d",px,py);
	//if(myDebug)
	//sprintf(ped_str,"sx%02dy%02d",1,10);
	

	int ry = getYGrid(rob_y);
	char rob_str[10];
	sprintf(rob_str,"sR%02d",ry);
	
	//if(myDebug)
	//sprintf(rob_str,"sR%02d",currRobY);
	
	
	if(ry>Y_SIZE-2)
	{
		currRobSpeed=0;
		pp->SetSpeed(0,0);
		cout << "Robot Reached goal .. " << endl;
		statfile.close();
		exit(1);
	}
		
		
	
	double rvel_double =  currRobSpeed/dSpeed;
	int rvel= (int) rvel_double;
	//if(rvel_double < 0.0001 )
		//rvel = (int) rvel_double + 1; /// the int takes floor value
	//cout << "rvel_d " << rvel_double << " rvel_int " << rvel << endl; 
	
	//cout << "psg velocity " << pp->GetYSpeed() << endl;
		
	char rob_vel_str[10];
	sprintf(rob_vel_str,"sV%d",rvel);
	//cout << " rvel : " << rob_vel_str << endl;
	//if(myDebug)
	//sprintf(rob_vel_str,"sV%d",currRobVel);
	
	string state_str;
	state_str.append(ped_str);
	state_str.append(rob_str);
	state_str.append(rob_vel_str);

	/// Lookup State id
	StateVal = ObsStateMapping[state_str];

    cout << " Curr state " << state_str << " id " << StateVal << endl;	
	
	
	
	
	return StateVal;
}


int getCurrObs(int id)//SharedPointer<MOMDP>& problem)
{
	int ObsVal;
	/// Poll sensors
	/// get ped location

	char nameid[10];
	sprintf(nameid,"ped%d",id);


	double ped_x,ped_y,ped_yaw;		
	sp->GetPose2d(nameid, ped_x, ped_y, ped_yaw);
	
	
	int px = getXGrid(ped_x);
	int py = getYGrid(ped_y);
	char ped_str[10];
	sprintf(ped_str,"ox%02dy%02d",px,py);
	
	ObsVal = ObsSymbolMapping[ped_str];
	
	cout << "curr observation is " << ped_str << " id " << ObsVal << endl;
	
	/// Bin continuous values to get discrete values
	
	//map<string, string> bb = problem->getObservationsSymbols(2);
	//map<string, string>::iterator it = bb.begin();
	//cout << " curr obs " << (*it).first << " " << (*it).second << endl;
	
	//ObsVal = 2;
	
	return ObsVal;
}

void initPedGoal()
{

if(cross)
{
	GOAL G0 = { 5, Y_SIZE-1 };
	lPedGoal.push_back(G0);
	goalPt[0].px = 5; 
	goalPt[0].py = Y_SIZE-1;
	
	GOAL G1 = { X_SIZE-6, Y_SIZE-1 };
	lPedGoal.push_back(G1);
	goalPt[1].px = X_SIZE-6; 
	goalPt[1].py = Y_SIZE-1;

	GOAL G2 = { X_SIZE-1, Y_SIZE-4 };
	lPedGoal.push_back(G2);
	goalPt[2].px = X_SIZE-1; 
	goalPt[2].py = Y_SIZE -4;
	
	GOAL G3 = { 5, Y_SIZE-4 };
	lPedGoal.push_back(G3);

	//GOAL G0 = { X_SIZE-1, Y_SIZE/2 };
	//lPedGoal.push_back(G0);
	

	///// Stay behavior
	//GOAL G4;// =  { 0, Y_SIZE-1 };
	//lPedGoal.push_back(G4);


	pedGoal_x[0] = 5; 
	pedGoal_y[0] = Y_SIZE-1;
	
	pedGoal_x[1] = 5; 
	pedGoal_y[1] = Y_SIZE-1;

	pedGoal_x[2] = 5; 
	pedGoal_y[2] = Y_SIZE-1;

	pedGoal_x[3] = 5; 
	pedGoal_y[3] = Y_SIZE-1;
	
	goalPt[3].px = 5; 
	goalPt[3].py = Y_SIZE-1;
	
}
else if(lane)
{
	GOAL G0 = { 0, 0 };
	lPedGoal.push_back(G0);
	goalPt[0].px = 0; 
	goalPt[0].py = 0;
	
	GOAL G1 = { X_SIZE-1, 0 };
	lPedGoal.push_back(G1);
	goalPt[1].px = X_SIZE-1; 
	goalPt[1].py = 0;

	GOAL G2 = { X_SIZE-1, Y_SIZE-1 };
	lPedGoal.push_back(G2);
	goalPt[2].px = X_SIZE-1; 
	goalPt[2].py = Y_SIZE -1;
	
	GOAL G3 = { 0, Y_SIZE-1 };
	lPedGoal.push_back(G3);
	goalPt[3].px = 0; 
	goalPt[3].py = Y_SIZE -1;

	//GOAL G0 = { X_SIZE-1, Y_SIZE/2 };
	//lPedGoal.push_back(G0);
	

	///// Stay behavior
	//GOAL G4;// =  { 0, Y_SIZE-1 };
	//lPedGoal.push_back(G4);
	
}
	
	
}

void updateStats()
{

	double min_dist = 100;

	double rob_x,rob_y,rob_yaw;
	sp->GetPose2d("rob", rob_x, rob_y, rob_yaw);

	for(int id=0; id<NUM_PED; id++)
	{
		char nameid[10];
		sprintf(nameid,"ped%d",id);
			
		/// Poll sensors 

		double ped_x,ped_y,ped_yaw;	
		
		sp->GetPose2d(nameid, ped_x, ped_y, ped_yaw);
		
		//double dist_to_ped= sqrt( (rob_x-ped_x)*(rob_x-ped_x) + (rob_y-ped_y)*(rob_y-ped_y));
		double dist_to_ped = ped_y - rob_y;
		if(dist_to_ped>-0.5)
		if(min_dist > dist_to_ped)
			min_dist = dist_to_ped;

	}	
		
	if(min_dist !=100)
	statfile << " " << num_steps << " " << min_dist << " " << currRobSpeed << endl;
}

void getPolyPts(player_point_2d_t* pts2, double x, double y, double size)
{
	//player_point_2d_t pts;
	//x=0;y=-dY;
	double x_min = x - size;
	double y_min = y - size;
	double x_max = x + size;
	double y_max = y + size;
	
	/// pts in CCW
	pts2[0].px = x_min;
	pts2[0].py = y_min;

	pts2[1].px = x_min;
	pts2[1].py = y_max;

	pts2[2].px = x_max;
	pts2[2].py = y_max;

	pts2[3].px = x_min;
	pts2[3].py = y_max;


}

int main(int argc, char **argv) 
{
	fstream configfile;
	configfile.open("config.txt");
	
	configfile >> mostLikelyAlgo;// << configfile;
	
	initPedGoal();
			//int num_goal = lPedGoal.size();
		//player_point_2d_t goalPt[NUM_GOAL];
		//player_point_2d_t pedPt[NUM_PED];

	/// Some reason the global assignment is not working 
	dY = PSG_Y/Y_SIZE;
	dX = PSG_X/X_SIZE;
	//cout << " dY " << dY  << " PSG_Y " << PSG_Y << " Y_SIZE " << Y_SIZE << " dY:= " << PSG_Y/Y_SIZE << endl; 
	
	
		/// PSG Loop
		/// PSG Initialize 
		PlayerClient stageSimul("localhost", 6665);
		sp = new SimulationProxy(&stageSimul, 0);
		
		
		PlayerClient robot("localhost", 7001);
		pp = new Position2dProxy(&robot,0);
		pp->SetMotorEnable (true);
		//lp = new LaserProxy(&robot,0);

		for(int ii=0; ii<NUM_PED; ii++)
		{
			ped[ii] = new PlayerClient("localhost", (8000+ii));
			ped_pp[ii] = new Position2dProxy(ped[ii],0);
			ped_pp[ii]->SetMotorEnable (true);
			gp[ii] = new Graphics2dProxy(ped[ii], 0);

		}


		
	//if(NUM_PED>0)
	//{	
		//ped[0] = new PlayerClient("localhost", 8000);
		//ped_pp[0] = new Position2dProxy(ped[0],0);
		//ped_pp[0]->SetMotorEnable (true);
		//gp[0] = new Graphics2dProxy(ped[0], 0);
	//}
		
	////if(NUM_PED>1)
	////{
		//ped[1] = new PlayerClient("localhost", 8000);
		//ped_pp[1] = new Position2dProxy(&ped1,0);
		//ped_pp[1]->SetMotorEnable (true);
		//gp[1] = new Graphics2dProxy(&ped1, 0);
	////}

	////if(NUM_PED>2)
	////{
		//PlayerClient ped2("localhost", 8002);
		//ped_pp[2] = new Position2dProxy(&ped2,0);
		//ped_pp[2]->SetMotorEnable (true);
		//gp[2] = new Graphics2dProxy(&ped2, 0);
	////}

	////if(NUM_PED>3)
	////{
		//PlayerClient ped3("localhost", 8003);
		//ped_pp[3] = new Position2dProxy(&ped3,0);
		//ped_pp[3]->SetMotorEnable (true);
		//gp[3] = new Graphics2dProxy(&ped3, 0);
		
	////}	
		//sp = new SimulationProxy(&stageSimul, 0);
//// for initialization
//{
//double rob_x,rob_y,rob_yaw;
//double ped_x,ped_y,ped_yaw;		
//sp->GetPose2d("rob", rob_x, rob_y, rob_yaw);
//sp->GetPose2d("ped0", ped_x, ped_y, ped_yaw);
	
//cout << " rob " << rob_x << "," << rob_y << "," << rob_yaw << endl;
//cout << " ped " << ped_x << "," << ped_y << "," << ped_yaw << endl;
////exit(1);
//}		

int px_i,py_i,rx_i;
	if(cross)
	{
		px_i = 5;
		 py_i = 5;
		 rx_i=7;
	 }
	 else if(lane)
	 {
		px_i = 0;
		 py_i = 4;
		 rx_i=1;
	 }
	configfile >> px_i;
	configfile >> py_i;
	configfile >> YXSwitch;
	configfile >> XYSwitch;
	//rx_i << configfile;

		double px_init = getXPos(px_i);
		double py_init = getYPos(py_i);
		
		cout << " px " << px_init << " py " << py_init << endl;
		//exit(1); 


		double rx_init = getXPos(rx_i);
		double ry_init = getYPos(0);

	char name[50];
	char type[10];
	if(mostLikelyAlgo)
		sprintf(type,"ML");
	else
		sprintf(type,"momdp");
		
		
	if(lane)
		sprintf(name,"stats-lane-%s-%d-%d-r%d-xy%d-yx%d.dat",type,px_i,py_i,rx_i,XYSwitch,YXSwitch);
	else if(cross)
		sprintf(name,"stats-cross-%s-%d-%d-r%d-xy%d-yx%d.dat",type,px_i,py_i,rx_i,XYSwitch,YXSwitch);
	else
	{
		cout << " unknown environment " << endl;	
		exit(1);
	}
	statfile.open (name);

		
		
		sp->SetPose2d("rob", rx_init, ry_init, 1.57);
		
		if(NUM_PED>0)
		sp->SetPose2d("ped0", px_init, py_init, 0.0); //x00y06
				
		if(NUM_PED>1)
		sp->SetPose2d("ped1", getXPos(5), getYPos(2), 0.0); //x00y06
		
		if(NUM_PED>2)
		sp->SetPose2d("ped2", getXPos(5), getYPos(0), 0.0); //x00y06
		
		if(NUM_PED>3)
		sp->SetPose2d("ped3", getXPos(9), getYPos(0), 0.0); //x00y06
		
		sleep(SLEEP);

		

    try
    {
        SolverParams* p = &GlobalResource::getInstance()->solverParams;

        bool parseCorrect = SolverParams::parseCommandLineOption(argc, argv, *p);
        if(!parseCorrect)
        {
            print_usage(p->cmdName); 
            exit(EXIT_FAILURE);
        }

        //check validity of options
        if (p->policyFile == "" || p->simLen == -1 || p->simNum == -1) 
        {
            print_usage(p->cmdName); 
            return 0;
        }


        bool enableFiling = false;
        if (p->outputFile.length() == 0) 
        {
            enableFiling = false;
        } 
        else 
        {
            enableFiling = true;
        }

	cout << "\nLoading the model ..." << endl << "  ";
        SharedPointer<MOMDP> problem = ParserSelector::loadProblem(p->problemName, *p);

        if(p->stateMapFile.length() > 0 )
        {
            // generate unobserved state to variable value map
            ofstream mapFile(p->stateMapFile.c_str());
            for(int i = 0 ; i < problem->YStates->size() ; i ++)
            {
                //mapFile << "State : " << i <<  endl;
                map<string, string> obsState = problem->getFactoredUnobservedStatesSymbols(i);
                for(map<string, string>::iterator iter = obsState.begin() ; iter != obsState.end() ; iter ++)
                {
                    mapFile << iter->first << " : " << iter->second << endl ;
                }
            }
            mapFile.close();
        }

        SharedPointer<AlphaVectorPolicy> policy = new AlphaVectorPolicy(problem);

	cout << "\nLoading the policy ..." << endl;
	cout << "  input file   : " << p->policyFile << endl;
        bool policyRead = policy->readFromFile(p->policyFile);
        if(!policyRead)
        {
            return 0;
        }


	cout << "\nSimulating ..." << endl;
	
        if(p->useLookahead) /// TBP : always using one step look ahead
        {
            cout << "  action selection :  one-step look ahead" << endl;
        }
        else
        {
        }

        SimulationRewardCollector rewardCollector;
        rewardCollector.setup(*p);

        ofstream * foutStream = NULL;
        srand(p->seed);//Seed for random number.  Xan
        //cout << p->seed << endl;



        if (enableFiling) 
        {
            foutStream = new ofstream(p->outputFile.c_str());
        }



    
		/// Initialize MOMDP

			cout << "Settingup PSG_SimulationEngine " << endl;
            PSG_SimulationEngine engine;            
            engine.setup(problem, policy, p); /// TBP : p is params

            double reward = 0, expReward = 0;

            //int firstAction = engine.runFor(p->simLen, foutStream, reward, expReward);
            //if(firstAction < 0)
            //{
                //// something wrong happend, exit
                //return 0;
            //}

            //rewardCollector.addEntry(currSim, reward, expReward);
            //rewardCollector.printReward(currSim);

			/// Mapping states for quick reference
            for(int i = 0 ; i < problem->XStates->size() ; i ++)
            {
                //mapFile << "State : " << i <<  endl;
                //cout << "State : " << i <<  endl;
                map<string, string> obsState = problem->getFactoredObservedStatesSymbols(i);
                string state_str;
                for(map<string, string>::iterator iter = obsState.begin() ; iter != obsState.end() ; iter ++)
                {
                    //cout << iter->first << " : " << iter->second << endl ;
                    state_str.append(iter->second);
                }
                ObsStateMapping[state_str]=i;
            }
            /// Mapping observations for quick reference
			for(int i = 0 ; i < problem->observations->size() ; i ++)
            {
                //mapFile << "State : " << i <<  endl;
                //cout << "Observations : " << i <<  endl;
                map<string, string> obsSym = problem->getObservationsSymbols(i);
                string obs_str;
                for(map<string, string>::iterator iter = obsSym.begin() ; iter != obsSym.end() ; iter ++)
                {
                    //cout << iter->first << " : " << iter->second << endl ;
                    obs_str.append(iter->second);
                }
                ObsSymbolMapping[obs_str]=i;
            }
            
            
            
            
            
            
            
/// Initialize online execution

        /// policy follower state
        /// belief with state
		
        //SharedPointer<BeliefWithState> nextBelSt;
        //SharedPointer<BeliefWithState> currBelSt (new BeliefWithState());/// for policy follower based on known x value
									 /// set sval to -1 if x value is not known

        /// belief over x. May not be used depending on model type and commandline flags, but declared here anyways.
        //DenseVector currBelX; /// belief over x
        
        /// now choose a starting unobserved state for the actual system
        cout << "Initial state" << endl;
        //int currSVal = getCurrentState();
        
        //for(int ii=0; ii<NUM_PED; ii++)
        //currSVal[ii] = getCurrentState(ii);
        
        int currSVal[NUM_PED];
        vector< SharedPointer<BeliefWithState> > lcurrBelSt;
        int currAction[NUM_PED];

for(int ii=0; ii<NUM_PED; ii++)
{
        
        SharedPointer<BeliefWithState> currBelSt (new BeliefWithState());                
        
        currSVal[ii] = getCurrentState(ii);
        
        SharedPointer<SparseVector> startBeliefVec;
        if (problem->initialBeliefStval->bvec)
          startBeliefVec = problem->initialBeliefStval->bvec;
        else
          startBeliefVec = problem->initialBeliefYByX[currSVal[ii]];
	

	    /// TBP : initializing belief for Y
        int currUnobsState = chooseFromDistribution(*startBeliefVec); 
        int belSize = startBeliefVec->size();


        currBelSt->sval = currSVal[ii];
        copy(*currBelSt->bvec, *startBeliefVec);
        cout << "Starting Belief " << endl;
        currBelSt->bvec->write(cout);//, *streamOut);
        cout << endl;
        
        lcurrBelSt.push_back(currBelSt);
        currAction[ii] = policy->getBestActionLookAhead(*currBelSt); 
}    
    double mult=1;
    double gamma = problem->getDiscount();
    
    /// initial visualizer
    for(int kk=0; kk<NUM_GOAL; kk++)
    {
	player_color_t col2;
	memset( &col2, 0, sizeof(col2));
					
	col2.blue =255;// (int)(1 * 255.0);
    player_point_2d_t t;
    t.px = getXPos(goalPt[kk].px) - pedPt[0].px;		
	t.py = getYPos(goalPt[kk].py) - pedPt[0].py;
	player_point_2d_t pts1[4];
	double r=dX*0.5*1.0/NUM_GOAL;
	pts1[0].px = -r + t.px;
	pts1[0].py = -r + t.py;
	pts1[1].px = r + t.px;
	pts1[1].py = -r + t.py;
	pts1[2].px = r + t.px;
	pts1[2].py = r+ t.py;
	pts1[3].px = -r + t.px;
	pts1[3].py = r+ t.py;
	
	gp[0]->DrawPolygon (pts1, 4, 1, col2);
	}
	
    
       /// go into read-think-act loop
    for(;;)
    {
		cout << "=====================================================================" << endl;
		num_steps++;
		cout << "Curr State in loop : " << endl; //getCurrentState();/// Just to print current state	
      
      
		//int currAction = policy->getBestActionLookAhead(*currBelSt); 
		int mlcurrAction[NUM_PED];
		for(int ii=0; ii<NUM_PED; ii++)
		{
			currAction[ii] = policy->getBestActionLookAhead(*(lcurrBelSt[ii])); 
		
			if(mostLikelyAlgo)
			{
				cout << " Most Likely algorithm " << endl;
				int mostProbY  = (lcurrBelSt[ii])->bvec->argmax(); 	//get the most probable Y state
				double prob = (lcurrBelSt[ii])->bvec->operator()(mostProbY);	//get its probability

				SharedPointer<BeliefWithState> currMLBel (new BeliefWithState());
				//SparseVector currMLBel;
				//copy(currMLBel, currBelSt);
				copy(*currMLBel->bvec, *(lcurrBelSt[ii])->bvec);
				
				int resize = (lcurrBelSt[ii])->bvec->size();
				currMLBel->bvec->resize(resize);
				currMLBel->bvec->push_back(mostProbY, 1.0);
				currMLBel->sval = (lcurrBelSt[ii])->sval;
				
				mlcurrAction[ii] = policy->getBestActionLookAhead(*currMLBel); 
				
				if(mlcurrAction[ii]==currAction[ii])
				{
					cout << "Same actions " << endl;				
				}
				else 
					currAction[ii] = mlcurrAction[ii];
			}
		}
		
		/// combining the actions by picking the safest action
		
		int safeAction=1; /// actions : cru=0, acc=1, decc=2
		for(int ii=0; ii<NUM_PED; ii++)
		{
			if( (currAction[ii]!=safeAction) && (safeAction!=2) )
			{
				if(currAction[ii]==2)
					safeAction = 2;
				else if (currAction[ii]==0)
					safeAction =0;
			}			
		}				
				
		
		//map<string, string> aa = problem->getActionsSymbols(currAction);
		map<string, string> aa = problem->getActionsSymbols(safeAction);
		cout << "safe action " << aa["action_robot"] << endl;	
		
		if(moveRob)
		executeActions(aa["action_robot"]);
		else
		{
		pp->SetSpeed(0,0); sleep(SLEEP);
		}
		
		if(movePed)
		{
			for(int ii=0; ii<NUM_PED; ii++)
			MovePed(ii);
			//MovePedGoTo(ii, pedGoal_x[ii], pedGoal_y[ii]);
			//MovePed_step(ii);
		}
	
		for(int ii=0; ii<NUM_PED; ii++)
			gp[ii]->Clear();
		
		for(int ii=0; ii<NUM_PED; ii++)
		{		
			cout << "---- subproblem update  #" << ii << " ---- " << endl;
				cout << "Next state " << endl;
				int nextSVal = getCurrentState(ii);
				int currObservation = getCurrObs(ii);
				
				//double currReward = engine.getReward(*currBelSt, currAction);		
							//expReward += mult*currReward;
					//mult *= gamma;
					//reward += currReward;
					
					//cout << "CurrReward " << currReward << " ExpReward " << expReward << endl;

				
				
				cout << "before update belief" << endl;
				(lcurrBelSt[ii])->bvec->write(cout); cout << endl;
				cout << "curr bel sval " << (lcurrBelSt[ii])->sval << endl;
				
				SharedPointer<BeliefWithState> nextBelSt;
				engine.runStep((lcurrBelSt[ii]), safeAction, currObservation, nextSVal, nextBelSt );
				
				copy(*(lcurrBelSt[ii])->bvec, *nextBelSt->bvec);
					(lcurrBelSt[ii])->sval = nextSVal;

				cout << "next belief" << endl;
				(lcurrBelSt[ii])->bvec->write(cout); cout << endl;
				cout << "next bel sval " << (lcurrBelSt[ii])->sval << endl;
				
				/// visualize the beliefs	
				
				player_point_2d_t pts[2];
				//pts[0] = pedPt[ii];
				pts[0].px = 0;//getXPos(5);
				pts[0].py = 0;//getYPos(5);		
				
				//std::vector<SparseVector_Entry>::const_iterator iter = (lcurrBelSt[ii])->bvec->data.begin();
				
				for(int kk=0; kk<NUM_GOAL; kk++)
				{
					pts[1].px = getXPos(goalPt[kk].px) - pedPt[ii].px;		
					pts[1].py = getYPos(goalPt[kk].py) - pedPt[ii].py;		
								
					double dist = sqrt(pts[1].px*pts[1].px + pts[1].py*pts[1].py);
					
					//pts[1].px = getXPos(9) - pedPt[ii].px;		
					//pts[1].py = getYPos(7) - pedPt[ii].py;		

					//pts[1].px = getXPos(5+kk);
					//pts[1].py = getYPos(3);

					
					player_color_t col;
					memset( &col, 0, sizeof(col));
					double prob=0;
					for(std::vector<SparseVector_Entry>::const_iterator iter = (lcurrBelSt[ii])->bvec->data.begin(); iter != (lcurrBelSt[ii])->bvec->data.end(); iter++)
					{
						if(iter->index == kk)
						{
							prob = iter->value;
							col.blue = (int)(prob * 255.0);
							//col.red  = (int)(255.0 - prob * 255.0);
							
							col.alpha=0;
							//col.red=(int) 200*prob;
							//col.green=200;
							//col.blue=200;
							cout << "found iter->index " << kk << endl;
							break;
						}
						else
							cout << "iter->index " << iter->index << " kk " << kk  << endl;
					}
					
					//double alpha = (lcurrBelSt[ii])->bvec->operator()(kk); /// current belief about this goal
					gp[ii]->Color( (int)255.0*(1-prob),0,255, 1 );	// r,g,b,alpha
					//gp[ii]->Color(col);
					
					player_color_t col2;
					memset( &col2, 0, sizeof(col2));
					
					col2.blue =255;// (int)(1 * 255.0);
					//col2.red  = (int)(255.0 - 1 * 255.0);
					//gp[ii]->Color( 255,0,0, 0 );	
					if( prob >0.0001)
					{			
						//gp[ii]->DrawPolyline( pts, 2 );
						player_point_2d_t polypts[4];
	//double size=dX*prob;
	//double x_min = pts[1].px - size;
	//double y_min = pts[1].py - size;
	//double x_max = pts[1].px + size;
	//double y_max = pts[1].py + size;
	
	///// pts in CCW doesnt work use CW
	//polypts[0].px = x_min;
	//polypts[0].py = y_min;

	//polypts[1].px = x_min;
	//polypts[1].py = y_max;

	//polypts[2].px = x_max;
	//polypts[2].py = y_max;

	//polypts[3].px = x_min;
	//polypts[3].py = y_max;
	
	player_point_2d_t pts1[4];
	double r=dX*0.5*prob;
		pts1[0].px = -r + pts[1].px;
	pts1[0].py = -r + pts[1].py;
	pts1[1].px = r + pts[1].px;
	pts1[1].py = -r + pts[1].py;
	pts1[2].px = r + pts[1].px;
	pts1[2].py = r+ pts[1].py;
	pts1[3].px = -r + pts[1].px;
	pts1[3].py = r+ pts[1].py;
	
	gp[ii]->DrawPolygon (pts1, 4, 1, col2);
	
						//getPolyPts(polypts,  pts[1].px, pts[1].py,  prob*size);
						//getPolyPts(polypts,  pedPt[ii].px, pedPt[ii].py,  size);
						//if(col.alpha)
						//gp[ii]->DrawPolygon (pts1, 4, 1, col2);
						//cout << "  drawing polygon " << kk << " at " << getXGrid(pts[1].px) << ", " << getYGrid(pts[1].py)<< endl;
						//cout << "  drawing polygon " << kk << " at " << goalPt[kk].px << ", " << goalPt[kk].py<< endl;

						//sleep(1);
						//usleep(300000);
						
						//void 	DrawPolygon (player_point_2d_t pts[], int count, bool filled, player_color_t fill_color)
					}
					else
						cout << " cannot draw polygon " << kk << endl;
				}
				//sleep(1);


		}
		updateStats();
	} /// end unlimited for
        

        if (enableFiling) 
        {
            foutStream->close();
        }

        rewardCollector.printFinalReward();
        DEBUG_LOG( generateSimLog(*p, rewardCollector.globalExpRew, rewardCollector.confInterval); );

    }
    catch(bad_alloc &e)
    {
        if(GlobalResource::getInstance()->solverParams.memoryLimit == 0)
        {
            cout << "Memory allocation failed. Exit." << endl;
        }
        else
        {
            cout << "Memory limit reached. Please try increase memory limit" << endl;
        }

    }
    catch(exception &e)
    {
        cout << "Exception: " << e.what() << endl ;
    }
    return 0;
}

