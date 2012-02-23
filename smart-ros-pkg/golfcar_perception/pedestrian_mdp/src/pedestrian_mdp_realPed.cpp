/*
 * pedestrian_mdp.cpp
 *
 *  Created on: Sep 15, 2011
 *      Author: golfcar
 */

#include "pedestrian_mdp_realPed.h"

using namespace std;
void pedestrian_mdp::initPedGoal()
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
	//lPedGoal.push_back(G0);timer_ = n.createTimer(ros::Duration(0.05), &PurePursuit::controlLoop, this);


	///// Stay behavior
	//GOAL G4;// =  { 0, Y_SIZE-1 };
	//lPedGoal.push_back(G4);



}


pedestrian_mdp::pedestrian_mdp(int argc, char** argv) {
	ros::NodeHandle nh;
	robotSub_ = nh.subscribe("base_pose_ground_truth", 1, &pedestrian_mdp::robotPoseCallback, this);
	pedSub_ = nh.subscribe("single_pedestrian_cloud", 1, &pedestrian_mdp::pedPoseCallback, this);
	//subscribe as heartbeat of the robot
	//scanSub_ = nh.subscribe("clock", 1, &pedestrian_mdp::scanCallback, this);
	cmdPub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

	//pomdp ini


	initPedGoal();
			//int num_goal = lPedGoal.size();
		//player_point_2d_t goalPt[NUM_GOAL];
		//player_point_2d_t pedPt[NUM_PED];

	/// Some reason the global assignment is not working
//	dY = PSG_Y/Y_SIZE;
//	dX = PSG_X/X_SIZE;
	//cout << " dY " << dY  << " PSG_Y " << PSG_Y << " Y_SIZE " << Y_SIZE << " dY:= " << PSG_Y/Y_SIZE << endl;




        p = &GlobalResource::getInstance()->solverParams;

        bool parseCorrect = SolverParams::parseCommandLineOption(argc, argv, *p);
        if(!parseCorrect)
        {
            //print_usage(p->cmdName);
            exit(EXIT_FAILURE);
        }

        //check validity of options
        if (p->policyFile == "" || p->simLen == -1 || p->simNum == -1)
        {
            //print_usage(p->cmdName);
            return;
        }


	cout << "\nLoading the model ..." << endl << "  ";
        problem = ParserSelector::loadProblem(p->problemName, *p);

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

        policy = new AlphaVectorPolicy(problem);

	cout << "\nLoading the policy ..." << endl;
	cout << "  input file   : " << p->policyFile << endl;
        bool policyRead = policy->readFromFile(p->policyFile);
        if(!policyRead)
        {
            return;
        }


	cout << "\nSimulating ..." << endl;

        if(p->useLookahead) /// TBP : always using one step look ahead
        {
            cout << "  action selection :  one-step look ahead" << endl;
        }
        else
        {
        }







		/// Initialize MOMDP

			cout << "Settingup PSG_SimulationEngine " << endl;

            engine.setup(problem, policy, p); /// TBP : p is params

            //double reward = 0, expReward = 0;

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

        for(int ii=0; ii<NUM_PED; ii++)
        currSVal[ii] = getCurrentState(ii);



    mult=1;
    gamma = problem->getDiscount();
    timer_ = nh.createTimer(ros::Duration(0.5), &pedestrian_mdp::controlLoop, this);
}

pedestrian_mdp::~pedestrian_mdp() {

}

void pedestrian_mdp::robotPoseCallback(nav_msgs::Odometry odo)
{
	robotx_ = odo.pose.pose.position.x;
	roboty_ = odo.pose.pose.position.y;
	robotspeedx_ = odo.twist.twist.linear.x;
}

void pedestrian_mdp::pedPoseCallback(sensor_msgs::PointCloud pc)
{
	pedx_ = -pc.points[0].y;
	pedy_ = pc.points[0].x;
	//cout<<pedx_<<' '<<pedy_<<endl;
	obs_flag = true;
}

void pedestrian_mdp::controlLoop(const ros::TimerEvent &e)
{


	if(!obs_flag)return;
	if(obs_first)
	{
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
		obs_first = false;
		return;
	}
	//Start mdp stuff
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
	{

		if(safeAction==0) cmd.linear.x = 0;
		else if(safeAction==1) cmd.linear.x += 0.5;
		else if(safeAction==2) cmd.linear.x -= 0.5;
		if(cmd.linear.x<=0) cmd.linear.x = 0;
		if(cmd.linear.x>=1.5) cmd.linear.x = 1.5;
		cmdPub_.publish(cmd);
	}


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
		//ROS_INFO << "next bel sval " << (lcurrBelSt[ii])->sval << endl;
		ROS_INFO ("next bel sval %d", (lcurrBelSt[ii])->sval);

	}
	//updateStats();
}

int pedestrian_mdp::getCurrObs(int id)//SharedPointer<MOMDP>& problem)
{
	int ObsVal;
	/// Poll sensors
	/// get ped location

	int px = getXGrid(pedx_);
	int py = getYGrid(pedy_);
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

int pedestrian_mdp::getCurrentState(int id)
{
	int StateVal;

	/// Poll sensors
	double rob_x,rob_y,rob_yaw;
//	double ped_x,ped_y,ped_yaw;

	//sp->GetPose2d("rob", rob_x, rob_y, rob_yaw);
	//sp->GetPose2d(nameid, ped_x, ped_y, ped_yaw);
	rob_x = robotx_;
	rob_y = roboty_;
	currRobSpeed = robotspeedx_;

	cout<<rob_x<<' '<<rob_y<<currRobSpeed<<endl;
	//pedPt[id].px = pedx_;
	//pedPt[id].py = pedy_;

//ped sx00y02
//rob sR01
//rvel sV0
// ---
//State : 5098
//PedPose_0 : sx03y16
//RobPose_0 : str
//RobVel_0 : sV2


	/// Bin continuous values to get discrete values

	int px = getXGrid(pedx_);
	int py = getYGrid(pedy_);
	char ped_str[30];
	sprintf(ped_str,"sx%02dy%02d",px,py);
	//if(myDebug)
	//sprintf(ped_str,"sx%02dy%02d",1,10);


	int ry = getYGrid(rob_y);
	char rob_str[30];
	sprintf(rob_str,"sR%02d",ry);

	//if(myDebug)
	//sprintf(rob_str,"sR%02d",currRobY);

	cout<<"rob_str"<<endl;
	if(ry>Y_SIZE-1)
	{
		exit(1);
	}


	//dj: discretization of robot speed into 3 int levels (0,1,2)
	double rvel_double =  currRobSpeed/1.5*2;//dSpeed;
	int rvel= (int) rvel_double;
	//if(rvel_double < 0.0001 )
		//rvel = (int) rvel_double + 1; /// the int takes floor value
	//cout << "rvel_d " << rvel_double << " rvel_int " << rvel << endl;

	//cout << "psg velocity " << pp->GetYSpeed() << endl;

	char rob_vel_str[30];
	sprintf(rob_vel_str,"sV%d",rvel);

	cout << " rvel : " << rob_vel_str << endl;
	//if(myDebug)
	//sprintf(rob_vel_str,"sV%d",currRobVel);

	string state_str;
	state_str.append(ped_str);
	state_str.append(rob_str);
	state_str.append(rob_vel_str);
	cout<<" Before state lookup"<<endl;
	/// Lookup State id
	StateVal = ObsStateMapping[state_str];

    cout << " Curr state " << state_str << " id " << StateVal << endl;




	return StateVal;
}

int pedestrian_mdp::getXGrid(double x)
{
	int px = (int) (x+X_OFFSET)/dX ;
	return px;
}

int pedestrian_mdp::getYGrid(double y)
{
	int py = (int) (y+Y_OFFSET)/dY ;
	return py;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mdp");
	pedestrian_mdp mdp_node(argc, argv);
	ros::spin();
}
