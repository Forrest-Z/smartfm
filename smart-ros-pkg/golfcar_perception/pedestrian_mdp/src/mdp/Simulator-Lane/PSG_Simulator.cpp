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








#define X_SIZE 4
#define Y_SIZE 10

bool myDebug=false;
//int currRobY;
//int currRobVel;

#include "args.h"

SimulationProxy* sp;
Position2dProxy* pp;



#define SLEEP 1

//double dY=64.0/Y_SIZE;
//double dX=16.0/X_SIZE;
//double Y_OFFSET=32;
//double X_OFFSET=8;

double PSG_Y=60.0;
double PSG_X=14.0;

double dY= PSG_Y/Y_SIZE; /// step size in Y
double dX= PSG_X/X_SIZE; /// step size in X
double Y_OFFSET=PSG_Y/2.0;  /// getting zero to bot-left
double X_OFFSET=PSG_X/2.0;

double dSpeed=dY; /// Acc 
double currRobSpeed=0;
#define MAX_ROB_SPEED 2*dY

map<string, int> ObsStateMapping;
map<string, int> ObsSymbolMapping;

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
	return dx;
}

double getYPos(int y)
{
	double dy = (y) * dY -Y_OFFSET +0.5*dY;
	return dy;
}


void executeActions(string action)
{
	/// execute the action
	
	double rob_x,rob_y,rob_yaw;		
	sp->GetPose2d("rob", rob_x, rob_y, rob_yaw);
	
	double ped_x,ped_y,ped_yaw;	
	sp->GetPose2d("ped", ped_x, ped_y, ped_yaw);
	
	///-------------
	
	int px = getXGrid(ped_x);
	
	int py = getYGrid(ped_y);
	int ry = getYGrid(rob_y);
	
	
	cout << " Current px " << ped_x << ":" << px << " py " << ped_y <<":" << py << " ry " << rob_y << ":" << ry << endl;
	
	
	//if(py>Y_SIZE)
		//exit(1);

	if(ry>Y_SIZE-1)
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
			currRobSpeed = currRobSpeed+dSpeed;
		
		if(currRobSpeed > MAX_ROB_SPEED)
			currRobSpeed = MAX_ROB_SPEED;
		
		if(myDebug)
			sp->SetPose2d("rob", rob_x, rob_y+currRobSpeed, 0.0);
		else
			pp->SetSpeed(currRobSpeed,0);
		
	}
	else if(strcmp(action.c_str(),"aDec")==0)
	{
		if(currRobSpeed >0)
			currRobSpeed = currRobSpeed - dSpeed;
			
		if(currRobSpeed<0)
			currRobSpeed =0;
		
		if(myDebug)
			sp->SetPose2d("rob", rob_x, rob_y+currRobSpeed, 0.0);
		else
			pp->SetSpeed(currRobSpeed,0);
		
	}
	else if(strcmp(action.c_str(),"aCru")==0)
	{
		/// cruising

		if(myDebug)
			sp->SetPose2d("rob", rob_x, rob_y+currRobSpeed, 0.0);
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
	
	{
		double rob_x,rob_y1,rob_yaw;		
	sp->GetPose2d("rob", rob_x, rob_y1, rob_yaw);
	
	double ped_x,ped_y,ped_yaw;	
	sp->GetPose2d("ped", ped_x, ped_y, ped_yaw);
	
	///-------------
	
	int px = getXGrid(ped_x);
	
	int py = getYGrid(ped_y);
	int ry = getYGrid(rob_y1);	
		cout << " Next px " << ped_x << ":" << px << " py " << ped_y <<":" << py << " ry " << rob_y1 << ":" << ry << endl;
}
	return;
}

bool flag=true;
void MovePed()
{
	double ped_x,ped_y,ped_yaw;	
	sp->GetPose2d("ped", ped_x, ped_y, ped_yaw);
	
	//flag=false; /// horizontal
	//flag=true; /// vertical
	if(flag)
	{
		/// move vertical	
		if( getYGrid(	ped_y+dY) < Y_SIZE)
		{
			sp->SetPose2d("ped", ped_x, ped_y+dY, ped_yaw);
			flag = false;
		}
	}
	else
	{
		/// move horizontally
		if( getXGrid(	ped_x+dX) < X_SIZE)
		{
			sp->SetPose2d("ped", ped_x+dX, ped_y, ped_yaw);
			flag = true;		
		}
	}
	
	sleep(SLEEP);
}

int getCurrentState()
{
	
	int StateVal;
	
	/// Poll sensors 
	double rob_x,rob_y,rob_yaw;
	double ped_x,ped_y,ped_yaw;	
	
	sp->GetPose2d("rob", rob_x, rob_y, rob_yaw);
	sp->GetPose2d("ped", ped_x, ped_y, ped_yaw);
	
	cout << " rob " << rob_x << "," << rob_y << "," << rob_yaw << endl;
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
	
	
	if(ry>Y_SIZE-1)
	{
		currRobSpeed=0;
		pp->SetSpeed(0,0);
		cout << "Robot Reached goal .. " << endl;
		exit(1);
	}
		
		
	
	int rvel = (int) currRobSpeed/dSpeed;
	char rob_vel_str[10];
	sprintf(rob_vel_str,"sV%d",rvel);
	cout << " rvel : " << rob_vel_str << endl;
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

int getCurrObs()//SharedPointer<MOMDP>& problem)
{
	int ObsVal;
	/// Poll sensors
	/// get ped location

	double ped_x,ped_y,ped_yaw;	
	
	sp->GetPose2d("ped", ped_x, ped_y, ped_yaw);
	
	
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


int main(int argc, char **argv) 
{
		/// PSG Loop
		/// PSG Initialize 
		PlayerClient stageSimul("localhost", 6665);
		sp = new SimulationProxy(&stageSimul, 0);
		
		PlayerClient robot("localhost", 7001);
		pp = new Position2dProxy(&robot,0);
		pp->SetMotorEnable (true);
		//sp = new SimulationProxy(&stageSimul, 0);

		
		double px_init = getXPos(0);
		double py_init = getYPos(5);

		double rx_init = getXPos(1);
		double ry_init = getYPos(0);


		
		
		sp->SetPose2d("rob", rx_init, ry_init, 1.57);
		sp->SetPose2d("ped", px_init, py_init, 0.0); //x00y06
		//sp->SetPose2d("ped", -4.727, 0.0, 0.000); 
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
                mapFile << "State : " << i <<  endl;
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
                cout << "State : " << i <<  endl;
                map<string, string> obsState = problem->getFactoredObservedStatesSymbols(i);
                string state_str;
                for(map<string, string>::iterator iter = obsState.begin() ; iter != obsState.end() ; iter ++)
                {
                    cout << iter->first << " : " << iter->second << endl ;
                    state_str.append(iter->second);
                }
                ObsStateMapping[state_str]=i;
            }
            /// Mapping observations for quick reference
			for(int i = 0 ; i < problem->observations->size() ; i ++)
            {
                //mapFile << "State : " << i <<  endl;
                cout << "Observations : " << i <<  endl;
                map<string, string> obsSym = problem->getObservationsSymbols(i);
                string obs_str;
                for(map<string, string>::iterator iter = obsSym.begin() ; iter != obsSym.end() ; iter ++)
                {
                    cout << iter->first << " : " << iter->second << endl ;
                    obs_str.append(iter->second);
                }
                ObsSymbolMapping[obs_str]=i;
            }
            
            
            
            
            
            
            
/// Initialize online execution

        /// policy follower state
        /// belief with state
        SharedPointer<BeliefWithState> nextBelSt;
        SharedPointer<BeliefWithState> currBelSt (new BeliefWithState());/// for policy follower based on known x value
									 /// set sval to -1 if x value is not known

        /// belief over x. May not be used depending on model type and commandline flags, but declared here anyways.
        //DenseVector currBelX; /// belief over x
        
        /// now choose a starting unobserved state for the actual system
        
        int currSVal = getCurrentState();
        
        SharedPointer<SparseVector> startBeliefVec;
        if (problem->initialBeliefStval->bvec)
          startBeliefVec = problem->initialBeliefStval->bvec;
        else
          startBeliefVec = problem->initialBeliefYByX[currSVal];
	

	    /// TBP : initializing belief for Y
        int currUnobsState = chooseFromDistribution(*startBeliefVec); 
        int belSize = startBeliefVec->size();


        currBelSt->sval = currSVal;
        copy(*currBelSt->bvec, *startBeliefVec);
        cout << "Starting Belief " << endl;
        currBelSt->bvec->write(cout);//, *streamOut);
        cout << endl;
    
    double mult=1;
    double gamma = problem->getDiscount();
    
       /// go into read-think-act loop
    for(;;)
    {
      cout << "=====================================================================" << endl;
      
		int currAction = policy->getBestActionLookAhead(*currBelSt); 
		map<string, string> aa = problem->getActionsSymbols(currAction);
		cout << "curr action " << aa["action_robot"] << endl;		
		executeActions(aa["action_robot"]);
		MovePed();
		
		cout << "Next state " << endl;
		int nextSVal = getCurrentState();
		int currObservation = getCurrObs();
		
		double currReward = engine.getReward(*currBelSt, currAction);		
		            expReward += mult*currReward;
            mult *= gamma;
            reward += currReward;
            
            cout << "CurrReward " << currReward << " ExpReward " << expReward << endl;

		
		
		cout << "before update belief" << endl;
		currBelSt->bvec->write(cout); cout << endl;
		cout << "curr bel sval " << currBelSt->sval << endl;
		

		engine.runStep(currBelSt, currAction, currObservation, nextSVal, nextBelSt );
		
		copy(*currBelSt->bvec, *nextBelSt->bvec);
            currBelSt->sval = nextSVal;

		cout << "next belief" << endl;
		currBelSt->bvec->write(cout); cout << endl;
		cout << "next bel sval " << currBelSt->sval << endl;


 
	}
        

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

