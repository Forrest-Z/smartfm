/*
 * laserobstacleavoid.cc
 *
 * a simple obstacle avoidance demo
 *
 * @todo: this has been ported to libplayerc++, but not tested
 */

#include <libplayerc++/playerc++.h>
#include <iostream>


#include "SimulationEngine.h"
#include "GlobalResource.h"
#include "SimulationRewardCollector.h"
#include <string>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <ctime>
#include <map>
#include <vector>

#include "CPTimer.h"
#include "solverUtils.h"

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

//namespace momdp
//{
#define X_SIZE 4
#define Y_SIZE 20

bool myDebug=false;
int currRobY;
int currRobVel;

#include "args.h"

SimulationProxy* sp;
Position2dProxy* pp;
double currRobSpeed=0;
#define MAX_ROB_SPEED 3
double dSpeed=1.5; /// Acc 


double dY=64.0/Y_SIZE;
double dX=16.0/X_SIZE;
double Y_OFFSET=32;
double X_OFFSET=8;

map<string, int> ObsStateMapping;
map<string, int> ObsSymbolMapping;


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

	int px = (int) (ped_x+X_OFFSET)/dX;
	int py = (int) (ped_y+Y_OFFSET)/dY;
	char ped_str[10];
	sprintf(ped_str,"sx%02dy%02d",px,py);
	//if(myDebug)
	//sprintf(ped_str,"sx%02dy%02d",1,10);
	

	int ry = (int) (rob_y+Y_OFFSET)/dY;
	char rob_str[10];
	sprintf(rob_str,"sR%02d",ry);
	
	if(myDebug)
	sprintf(rob_str,"sR%02d",currRobY);
	
	
	if(ry>Y_SIZE)
	{
		currRobSpeed=0;
		pp->SetSpeed(0,0);
		exit(1);
	}
		
		
	
	int rvel = (int) currRobSpeed/dSpeed;
	char rob_vel_str[10];
	sprintf(rob_vel_str,"sV%d",rvel);
	if(myDebug)
	sprintf(rob_vel_str,"sV%d",currRobVel);
	
	string state_str;
	state_str.append(ped_str);
	state_str.append(rob_str);
	state_str.append(rob_vel_str);

	/// Lookup State id
	StateVal = ObsStateMapping[state_str];

    cout << " Curr state " << state_str << " id " << StateVal << endl;	
	
	
	
	
	return StateVal;
}

int getCurrObs(SharedPointer<MOMDP>& problem)
{
	int ObsVal;
	/// Poll sensors
	/// get ped location

	double ped_x,ped_y,ped_yaw;	
	
	sp->GetPose2d("ped", ped_x, ped_y, ped_yaw);
	
	
	int px = (int) (ped_x+X_OFFSET)/dX;
	int py = (int) (ped_y+Y_OFFSET)/dY;
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

SharedPointer<BeliefWithState> mynextBelief(SharedPointer<BeliefWithState> bp, int a, int o, int obsX, SharedPointer<MOMDP>& problem  )
{
	cout << "using this belief update .. tirtha 2" << endl;
	SparseVector jspv;
	SharedPointer<MOMDP> momdpProblem = dynamic_pointer_cast<MOMDP> (problem);
	// TODO:: should cache jspv somethere
	//momdpProblem->getJointUnobsStateProbVector(jspv, bp , a, obsX);
	momdpProblem->getJointUnobsStateProbVector(jspv, bp , 0, obsX);

	
	cout << " jspv " << endl;
	jspv.write(cout) << endl;

	SharedPointer<BeliefWithState> result (new BeliefWithState());
	emult_column( *result->bvec, *(momdpProblem->obsProb->getMatrix(a, obsX)), o,  jspv);
	(*result->bvec) *= (1.0/result->bvec->norm_1());
	
	result->sval = obsX;
	return result;

}

void printTuple(map<string, string> tuple, ofstream* streamOut){
	*streamOut << "(";
	for(map<string, string>::iterator iter = tuple.begin() ; iter != tuple.end() ; )
	{
	    *streamOut << iter->second;
	    if(++iter!=tuple.end())
		*streamOut << ",";
	}
	*streamOut<<")" << endl;
    }

void executeActions(string action)
{
	/// execute the action
	
	double rob_x,rob_y,rob_yaw;		
	sp->GetPose2d("rob", rob_x, rob_y, rob_yaw);
	
	if(strcmp(action.c_str(),"aAcc")==0)
	{
		if(currRobSpeed < MAX_ROB_SPEED)
		{
			currRobSpeed = currRobSpeed+dSpeed;
			//pp->SetSpeed(currRobSpeed,0);
			
				
		}
		sp->SetPose2d("rob", rob_x, rob_y+dY, 1.57);
		if(myDebug)
		{
			if(currRobVel<2)
				currRobVel++;
			currRobY= currRobY+currRobVel;
		}
	}
	else if(strcmp(action.c_str(),"aDec")==0)
	{
		if(currRobSpeed >0)
		{
			currRobSpeed = currRobSpeed - dSpeed;
			//pp->SetSpeed(currRobSpeed-dSpeed,0)
			sp->SetPose2d("rob", rob_x, rob_y+dY, 1.57);;
		}
		if(myDebug)
		{
			if(currRobVel>0)
				currRobVel--;
			currRobY= currRobY+currRobVel;
		}
	}
	else if(strcmp(action.c_str(),"aCru")==0)
	{
		// cruising
		if(myDebug)
			currRobY= currRobY+currRobVel;
		
	}
	else
	{
		cout << "Error executing actions .. " << endl;
		pp->SetSpeed(0,0);
		currRobSpeed=0;
	}
	
	sleep(1.5);
}

void performActionUnobs(belief_vector& outBelUnobs, int action, const BeliefWithState& belSt, int currObsState, SharedPointer<MOMDP>& problem ) 
  {
         SharedPointer<SparseMatrix>  transMatY = problem->YTrans->getMatrix(action, belSt.sval, currObsState);
        mult(outBelUnobs, *belSt.bvec, *transMatY);
  }
  
void performActionObs(belief_vector& outBelObs, int action, const BeliefWithState& belSt,SharedPointer<MOMDP>& problem  )  
    {
        // DEBUG_SIMSPEED_270409 skip calculating outprobs for x when there is only one possible x value
        if (problem->XStates->size() == 1) 
        {
            // clear out the entries
            outBelObs.resize(1);
            outBelObs.push_back(0,1.0); 
        } 
        else 
        {
            //problem->getTransitionMatrixX(action, belSt.sval);
	    const SharedPointer<SparseMatrix>  transMatX = problem->XTrans->getMatrix(action, belSt.sval); 
            mult(outBelObs, *belSt.bvec, *transMatX);
	}
    }


void getPossibleObservations(belief_vector& possObs, int action, 	const BeliefWithState& belSt, SharedPointer<MOMDP>& problem) 
    {
        //const SparseMatrix obsMat = problem->getObservationMatrix(action, belSt.sval);
		const SharedPointer<SparseMatrix>  obsMat = problem->obsProb->getMatrix(action, belSt.sval);
        mult(possObs,  *belSt.bvec, *obsMat);
    }
    
    
void print_usage(const char* cmdName) 
{
	cout << "Usage: " << cmdName << " POMDPModelFileName --policy-file policyFileName --simLen numberSteps \n" 
<<"	--simNum numberSimulations [--fast] [--srand randomSeed] [--output-file outputFileName]\n" 
<<"    or " << cmdName << " --help (or -h)  Print this help\n" 
<<"    or " << cmdName << " --version	  Print version information\n" 
<<"\n"
<<"Simulator options:\n"
<<"  --policy-file policyFileName	Use policyFileName as the policy file name (compulsory).\n"
<<"  --simLen numberSteps		Use numberSteps as the number of steps for each\n" 
<<"				simulation run (compulsory).\n"
<<"  --simNum numberSimulations	Use numberSimulations as the number of simulation runs\n" 
<<"				(compulsory).\n"
<<"  -f or --fast			Use fast (but very picky) alternate parser for .pomdp files.\n"
<<"  --srand randomSeed		Set randomSeed as the random seed for simulation.\n" 
<<"				It is the current time by default.\n"
//<<"  --lookahead yes/no		Set 'yes' ('no') to select action with (without) one-step\n" 
//<<"				look ahead. Action selection is with one-step look ahead\n" 
//<<"				by default.\n" 
<<"\n"
<<"Output options:\n"
<<"  --output-file outputFileName	Use outputFileName as the name for the output file\n" 
<<"				that contains the simulation trace.\n"
		<< "Example:\n"
		<< "  " << cmdName << " --simLen 100 --simNum 100 --policy-file out.policy Hallway.pomdp\n";

// 	cout << "usage: binary [options] problem:\n"
// 		<< "--help, print this message\n"
// 		<< "--policy-file, policy file to be used\n"
// 		<< "--output-file, output file to be used\n"
// 		<< "--simLen, length of simulation\n"
// 		<< "--simNum, number of simulations\n"
// 		<< "--srand, random seed (default: current time)\n"
// 		<< "--lookahead, use \"one-step look ahead\" when selecting action (default: yes)\n"
// 		<< "Examples:\n"
// 		<< " ./simulate --simLen 100 --simNum 100 --policy-file out.policy Hallway.pomdp\n";

}
 
int main(int argc, char** argv)
{

  //parse_args(argc,argv);
  /* initialize RANDOM seed: */
  srand ( time(NULL) );
   /// we throw exceptions on creation if we fail
  try
  {

    PlayerClient stageSimul("localhost", 6665);
    sp = new SimulationProxy(&stageSimul, 0);
    
	PlayerClient robot("localhost", 7001);
	pp = new Position2dProxy(&robot,0);
	pp->SetMotorEnable (true);
    //sp = new SimulationProxy(&stageSimul, 0);
    
	/// Initialize stuff
    sp->SetPose2d("rob", 0, -24.898, 1.57);
    sp->SetPose2d("ped", -4.727, -9.930, 0.000);


		SolverParams* p = &GlobalResource::getInstance()->solverParams;
				
		/// A dirty hack to generate the pomdp problem
		int inputArgc=10;
		//char inputArgv[][10] = "pomdpsim   --simLen 30 --simNum 1 --policy-file  output.policy --output-file output.out Ped-Road-simul.pomdpx ";
		//char inputArgv[][100] = {"pomdpsim"  ," --simLen",  "30", "--simNum",  "1"," --policy-file","  output.policy"," --output-file"," output.out"," Ped-Road-simul.pomdpx"};
		//cout << inputArgv[3] << endl;
		//bool parseCorrect = SolverParams::parseCommandLineOption(inputArgc, (char **)inputArgv, *p);
		bool parseCorrect = SolverParams::parseCommandLineOption(inputArgc, argv, *p);
		cout <<" print out the command name " <<  p->cmdName << endl;
		//p->cmdName = "pomdpsim";
		
		if(!parseCorrect)
        {
            print_usage(p->cmdName); 
            exit(EXIT_FAILURE);
        }
		
	cout << "\nLoading the model ..." << endl << "  ";
        SharedPointer<MOMDP> problem = ParserSelector::loadProblem(p->problemName, *p);
        //SharedPointer<MOMDP> problem = ParserSelector::loadProblem("Ped-Road-simul.pomdpx", *p);

        //if(p->stateMapFile.length() > 0 )
        //{
			//cout << "Entered here " << endl;
            // generate unobserved state to variable value map
            //ofstream mapFile(p->stateMapFile.c_str());
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
            
			for(int i = 0 ; i < problem->observations->size() ; i ++)
            {
                //mapFile << "State : " << i <<  endl;
                //cout << "State : " << i <<  endl;
                map<string, string> obsSym = problem->getObservationsSymbols(i);
                string obs_str;
                for(map<string, string>::iterator iter = obsSym.begin() ; iter != obsSym.end() ; iter ++)
                {
                    //cout << iter->first << " : " << iter->second << endl ;
                    obs_str.append(iter->second);
                }
                ObsSymbolMapping[obs_str]=i;
            }

            
            //mapFile.close();
        //}

        SharedPointer<AlphaVectorPolicy> policy = new AlphaVectorPolicy(problem);

	cout << "\nLoading the policy ..." << endl;
	//cout << "  input file   : " << p->policyFile << endl;
        bool policyRead = policy->readFromFile(p->policyFile);
        //bool policyRead = policy->readFromFile("output.policy");
        if(!policyRead)
        {
            return 0;
        }


	cout << "\nSimulating ..." << endl;
        //SimulationRewardCollector rewardCollector;
        //rewardCollector.setup(*p);
        

        /// actual system state
        SharedPointer<BeliefWithState> actStateCompl (new BeliefWithState());
        SharedPointer<BeliefWithState> actNewStateCompl (new BeliefWithState());

		/// policy follower state
        /// belief with state
        SharedPointer<BeliefWithState> nextBelSt;
        SharedPointer<BeliefWithState> currBelSt (new BeliefWithState());// for policy 
    
		/// initialize X state
		/// actStateCompl : actual complete state. X portion is from the real world
		//actStateCompl->sval = problem->initialBeliefStval->sval;
		actStateCompl->sval = getCurrentState();
		//currBelSt->sval = getCurrentState();
		
    
    
        /// now choose a starting unobserved state for the actual system
        SharedPointer<SparseVector> startBeliefVec;
        if (problem->initialBeliefStval->bvec)
          startBeliefVec = problem->initialBeliefStval->bvec;
        else
          startBeliefVec = problem->initialBeliefYByX[actStateCompl->sval];
	

	    /// TBP : initializing belief for Y
        int currUnobsState = chooseFromDistribution(*startBeliefVec); 
        int belSize = startBeliefVec->size();

        actStateCompl->bvec->resize(belSize);
        actStateCompl->bvec->push_back(currUnobsState, 1.0);

        currBelSt->sval = actStateCompl->sval;
        copy(*currBelSt->bvec, *startBeliefVec);
        currBelSt->bvec->write(cout);//, *streamOut);
        cout << endl;
    
    
    
    

    

    

    /// go into read-think-act loop
    for(;;)
    {
      cout << "------------------------------------------" << endl;
       
       /// Read the robot clients
       //robot.Read();
       

        /// we now have actStateCompl (starting stateUnobs&stateObs) for "actual state" system
		/// "policy follower" system has currBelSt (starting beliefUnobs&stateObs) OR 
		/// currBelSt (starting beliefUnobs&-1) and currBelX (starting beliefObs) if initial x is a belief and not a state

            //map<string, string> bb = problem->getObservationsSymbols(2);
            
            //for(map<string, string>::iterator it=bb.begin(); it!=bb.end(); it++)             
            //cout << " first " << (*it).first << " second " << (*it).second << endl;
            //map<string, string> cc = problem->getFactoredObservedStatesSymbols(25);//actStateCompl->sval);

            //for(map<string, string>::iterator it=cc.begin(); it!=cc.end(); it++)             
            //cout << " first " << (*it).first << " second " << cc["RobPose_0"] << endl;

        
        //actStateCompl->sval = getCurrentState();
        //currBelSt->sval = actStateCompl->sval;

		int currAction = policy->getBestActionLookAhead(*currBelSt); 
		map<string, string> aa = problem->getActionsSymbols(currAction);
		cout << "curr action " << aa["action_robot"] << endl;		
		executeActions(aa["action_robot"]);
    
            //// this is the reward for the "actual state" system
            //double currReward = getReward(*actStateCompl, currAction);

            //DEBUG_TRACE( cout << "currAction " << currAction << endl; );
            //DEBUG_TRACE( cout << "actStateCompl sval " << actStateCompl->sval << endl; );
            //DEBUG_TRACE( actStateCompl->bvec->write(cout) << endl; );

            //DEBUG_TRACE( cout << "currReward " << currReward << endl; );
            //expReward += mult*currReward;
            //mult *= gamma;
            //reward += currReward;

            //DEBUG_TRACE( cout << "expReward " << expReward << endl; );
            //DEBUG_TRACE( cout << "reward " << reward << endl; );
    
    
	    /// actualActionUpdObs is belief of the fully observered state
            //belief_vector actualActionUpdUnobs(belSize); 
            //belief_vector actualActionUpdObs(problem->XStates->size()) ;
            //performActionObs(actualActionUpdObs, currAction, *actStateCompl, problem); /// TBP : get real world obs
			//actualActionUpdObs.write(cout) << endl;
			
            //DEBUG_TRACE( cout << "actualActionUpdObs " << endl; );
            //DEBUG_TRACE( actualActionUpdObs.write(cout) << endl; );
            

            /// update state for the observed variable
            //string
            //actNewStateCompl->sval = getCurrentState();//+1;
            int nextState = getCurrentState();
            //nextBelSt->sval= actNewStateCompl->sval;//getCurrentState();
            //(unsigned int) chooseFromDistribution(actualActionUpdObs, ((double)rand()/RAND_MAX)); /// TBP : This is for moving the simulator world forward, we get this from real world.

	    /// now update actualActionUpdUnobs, which is the belief of unobserved states,
	    /// based on prev belief and curr observed state
			//performActionUnobs(actualActionUpdUnobs, currAction, *actStateCompl, actNewStateCompl->sval, problem); /// TBP : update bel Y
			//performActionUnobs(actualActionUpdUnobs, currAction, *currBelSt, currentState, problem); /// TBP : update bel Y
			
/// the actual next state for the unobserved variable
//int newUnobsState = chooseFromDistribution(actualActionUpdUnobs, ((double)rand()/RAND_MAX)); 
//DEBUG_TRACE( cout << "newUnobsState "<< newUnobsState << endl; );
//cout << "newUnobsState "<< newUnobsState << endl;

//currBelSt->bvec->write(cout);
//cout << "actionupdate " ;
//actualActionUpdUnobs.write(cout);
//cout << endl;

//copy(*currBelSt->bvec, actualActionUpdUnobs);
//cout << "after action update" ;
//currBelSt->bvec->write(cout);
//cout << endl;
//actNewStateCompl->bvec->resize(belSize);
//actNewStateCompl->bvec->push_back(newUnobsState, 1.0);

//DEBUG_TRACE( cout << "actNewStateCompl sval "<< actNewStateCompl->sval << endl; );
//DEBUG_TRACE( actNewStateCompl->bvec->write(cout) << endl; );
//cout << "actNewStateCompl sval "<< actNewStateCompl->sval << endl;
//actNewStateCompl->bvec->write(cout) << endl;

// get observations based on actual next states for observed and unobserved variable
//belief_vector obsPoss;
//getPossibleObservations(obsPoss, currAction, *actNewStateCompl,problem);  


//DEBUG_TRACE( cout << "obsPoss"<< endl; );
//DEBUG_TRACE( obsPoss.write(cout) << endl; );
//cout << "obsPoss"<< endl;
//obsPoss.write(cout); cout << endl;

//int currObservation = chooseFromDistribution(obsPoss, ((double)rand()/RAND_MAX)); /// TBP : get current observation from the real world			
//cout << " current observation " << currObservation << endl;
		
		/// get observation from sensors
			//int currObservation = getCurrObs( problem);
    
			//int record=false;
			//if(record)
			//{
				////initial belief Y state
					//int mostProbY  = currBelSt->bvec->argmax(); 	//get the most probable Y state
					//double prob = currBelSt->bvec->operator()(mostProbY);	//get its probability
					//streamOut->width(4);*streamOut<<left<<"ML Y"<<":";
							//map<string, string> mostProbYState = problem->getFactoredUnobservedStatesSymbols(mostProbY);
					//printTuple(mostProbYState, streamOut);
					
				///// -> TBP
					//streamOut->width(4);*streamOut << left << "BL Y"<<":(";
					////printTuple(currBelSt->bvec, *streamOut);
					////display(currBelSt->bvec, *streamOut);
					////currBelSt->bvec->write(*streamOut); *streamOut << endl;
					//*streamOut << currBelSt->bvec->data.size() <<",";
					//for(std::vector<SparseVector_Entry>::const_iterator iter = currBelSt->bvec->data.begin();iter != currBelSt->bvec->data.end(); iter++)
					//{
						////*streamOut << iter->value;
						//*streamOut << iter->index << "="<< iter->value;
						//if(iter < currBelSt->bvec->data.end()-1){
						//*streamOut <<", ";
						//}
						//else{
						//*streamOut << ")";
						//}
					//}
					//*streamOut << endl;
				///// <- TBP
			//}
    
			int currObservation = getCurrObs(problem);
			//nextBelSt = problem->beliefTransition->nextBelief(currBelSt, currAction, currObservation, actNewStateCompl->sval);
			cout << "before update belief" << endl;
			currBelSt->bvec->write(cout); cout << endl;
			cout << "curr bel sval" << currBelSt->sval << endl;
			//cout << "next bel before update " << endl;
			//nextBelSt = new(Belief  ->bvec->write(cout); cout << endl;
			
			///nextBelSt = problem->beliefTransition->nextBelief(currBelSt, currAction, currObservation, actNewStateCompl->sval);
			//nextBelSt = problem->beliefTransition->nextBelief(currBelSt, currAction, currObservation, actNewStateCompl->sval);
			SharedPointer<MOMDP> momdpProblem = dynamic_pointer_cast<MOMDP> (problem->beliefTransition->problem);
			//cout << "num actions for prob->belT->prob " <<   momdpProblem->getNumActions() << endl;
			//nextBelSt = problem->beliefTransition->nextBelief(currBelSt, currAction, currObservation, nextState);
			nextBelSt = mynextBelief(currBelSt, currAction, currObservation, nextState, momdpProblem);
			
			nextBelSt->bvec->resize(belSize);
			cout << "after update belief" << endl;			
			nextBelSt->bvec->write(cout); cout << endl;
    
    	    //actual states
            //currUnobsState = newUnobsState; //Y state, hidden
	    
	    //actStateCompl->sval = actNewStateCompl->sval;
            //copy(*actStateCompl->bvec, *actNewStateCompl->bvec);
	    
	    //belief states
            copy(*currBelSt->bvec, *nextBelSt->bvec);
            currBelSt->sval = nextState; //actNewStateCompl->sval;//nextBelSt->sval;
            //cout << " Current belief : "  <<endl;
            //currBelSt->bvec->write(cout);
            //display(currBelSt->bvec, cout);
            

		// added to stop simulation when at terminal state
            //if(problem->getIsTerminalState(*actStateCompl))
            //{
                //// Terminal state
                //// reward all 0, transfer back to it self
                //// TODO Consider reward not all 0 case
                ////cout <<"Terminal State!! timeIter " << timeIter << endl;
				////if(enableFiling)
					////*streamOut << "Reached terminal state" << endl;
					//cout << "Reached terminal state" << endl;
                //break;
            //}
       
       


    }// end of loop

  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << e << std::endl;
    return -1;
  }

}

//};
