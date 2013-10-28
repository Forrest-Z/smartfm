#include "momdp.h"

ped_momdp::ped_momdp(string model_file, string policy_file, int simLen, int simNum, bool stationary, double frequency, bool use_sim_time, ros::NodeHandle& nh)
{
    X_SIZE=30;
    Y_SIZE=6;
    dY= 3;// step size in Y
    dX= 3;// step size in X
    momdp_problem_timeout = 5.0;
    stationary_ = stationary;
    use_sim_time_ = use_sim_time;
    believesPub_ = nh.advertise<ped_momdp_sarsop::peds_believes>("peds_believes",1);
    cmdPub_ = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1);
    RobState_pub = nh.advertise<geometry_msgs::Twist>("pedState", 10);

    AnalysisCrash_pub = nh.advertise<std_msgs::Float32>("Analysis_Crash", 10);
	AnalysisTime_pub = nh.advertise<std_msgs::Float32>("Analysis_Time", 10); 

    timer_ = nh.createTimer(ros::Duration(1.0/frequency), &ped_momdp::controlLoopConstantGap2, this);

    policy_initialize(model_file, policy_file, simLen, simNum);

    analysisvar.data = 0;


}

ped_momdp::~ped_momdp()
{
    geometry_msgs::Twist cmd;
    cmd.angular.z = 0;
    cmd.linear.x = 0;
    cmdPub_.publish(cmd);
}

void ped_momdp::updateRobotSpeed(double speed)
{
    robotspeedx_ = speed;
}

void ped_momdp::addNewPed(ped_momdp_sarsop::ped_local_frame &ped)
{
    initPedMOMDP(ped);
}

bool ped_momdp::updatePedRobPose(ped_momdp_sarsop::ped_local_frame &ped)
{
	robotx_ =   ped.rob_pose.x;
	roboty_ =   ped.rob_pose.y;
    bool foundPed=false;
    for(int jj=0; jj< lPedInView.size(); jj++)
    {

        if(lPedInView[jj].id==ped.ped_id)
        {
            //given in ROS coordinate, convert to momdp compatible format
            lPedInView[jj].ped_pose.x = ped.ped_pose.x;
            lPedInView[jj].ped_pose.y = ped.ped_pose.y ;
            lPedInView[jj].rob_pose.x = ped.rob_pose.x;
            lPedInView[jj].rob_pose.y = ped.rob_pose.y;
            ///// Debug
            foundPed=true;
			lPedInView[jj].last_update = ros::Time::now();
            ROS_DEBUG_STREAM( "Updated ped #" << ped.ped_id << " " <<lPedInView[jj].ped_pose.x<<' '<<lPedInView[jj].ped_pose.y);
            break;

        }

    }
    return foundPed;
}

void ped_momdp::updateSteerAnglePublishSpeed(geometry_msgs::Twist speed)
{
    geometry_msgs::Twist cmd;
    cmd.angular.z = speed.angular.z;
    double momdp_speed = momdp_speed_;
    //need correction here for the speed compensation for simulation
    //fixed ros-pkg ticket #5432
    //if(use_sim_time_) momdp_speed = momdp_speed * 0.3; /// TBP change numbers into parameters

   //V_ if(speed.linear.x < momdp_speed_) cmd.linear.x = speed.linear.x;
   //V_ else cmd.linear.x = momdp_speed_;
    cmd.linear.x = momdp_speed_;

    geometry_msgs::Twist cmd_temp;
    cmd_temp = cmd;

    cmdPub_.publish(cmd_temp);
}
void ped_momdp::initPedMOMDP(ped_momdp_sarsop::ped_local_frame& ped_local)
{
    PED_MOMDP pedProblem;
    pedProblem.id = ped_local.ped_id;

    cout<<"create new belief state"<<endl;
    pedProblem.currBelSt = (new BeliefWithState());

    //cout<<"Ped pose: "<<ped_local.ped_pose<<endl;

    pedProblem.currSVal = getCurrentState(ped_local.rob_pose.x, ped_local.rob_pose.y, ped_local.ped_pose.x, ped_local.ped_pose.y);
    /// Debug
    //pedProblem.currSVal = getCurrentState(0, 0, -ped_local.ped_pose.y, ped_local.ped_pose.x);
    cout << "Current SVal " << pedProblem.currSVal << endl;

    SharedPointer<SparseVector> startBeliefVec;
    if (problem->initialBeliefStval->bvec)
        startBeliefVec = problem->initialBeliefStval->bvec;
    else
        startBeliefVec = problem->initialBeliefYByX[pedProblem.currSVal];



    pedProblem.currBelSt->sval = pedProblem.currSVal;
    copy(*(pedProblem.currBelSt)->bvec, *startBeliefVec);



    pedProblem.currAction = policy->getBestActionLookAhead(*(pedProblem.currBelSt));

    //Add newly observed pedestrian
    //V pedProblem.rob_pose = (double)ped_local.rob_pose.x;
    pedProblem.rob_pose.x = (double)ped_local.rob_pose.x;
    pedProblem.rob_pose.y = (double)ped_local.rob_pose.y;
    pedProblem.ped_pose.x = (double)ped_local.ped_pose.x;
    pedProblem.ped_pose.y = (double)ped_local.ped_pose.y;
    lPedInView.push_back(pedProblem);
}

/////////////////////CONTROL LOOP FOR MOMDP///////////////////////////////////////////////////////
void ped_momdp::controlLoopMOMDP(const ros::TimerEvent &e)
{

    cout << "Control loop dj .. " << " lPedInView " << lPedInView.size() << endl;


    if(lPedInView.size()==0)
    {
        momdp_speed_ = 3;
        return;
    }

    ///Start pomdp stuff
    cout << "=====================================================================" << endl;

    /// Get new action
    for(int ii=0; ii<lPedInView.size(); ii++)
    {
        lPedInView[ii].currAction = policy->getBestActionLookAhead(*(lPedInView[ii].currBelSt));
    }


    int safeAction=1;
        for(int ii=0; ii<lPedInView.size(); ii++)
        {
            if( lPedInView[ii].currAction == 0 )
            {
                   safeAction = 0;
            }
        }

    map<string, string> aa = problem->getActionsSymbols(safeAction);
    cout << "safe action " << aa["action_robot"] << endl;

    if(!stationary_)
    {
        /// TBP : Change numbers to parameters
        if(safeAction==0) momdp_speed_ = 0;
        else if(safeAction==1) momdp_speed_ = 3;

    }



    publish_belief();

    /// update belief based on the action taken
    for(int ii=0; ii<lPedInView.size(); ii++)
    {
        int id = lPedInView[ii].id;
        updateBelief(id,safeAction);
    }

    clean_momdp_problem();

}

/////////////////////CONTROL LOOP FOR CONSTANT GAP-2///////////////////////////////////////////////////////
void ped_momdp::controlLoopConstantGap2(const ros::TimerEvent &e)
{
	int safeAction=1;
	volkan.linear.x = robotx_;
	volkan.linear.y = roboty_;

	RobState_pub.publish(volkan);

    cout << "Control loop dj .. " << " lPedInView " << lPedInView.size() << endl;
    if(getYGrid(roboty_)<3 )//Approach to the intersection area without any other concern!
    {
    	safeAction=1;
    }
    else if(getYGrid(roboty_)>3 )//After merging the way, don't stop!Only condition you can stop is there is  a car in front of you is very near to you
    {		
    	safeAction=1;
    		for(int ii=0; ii<lPedInView.size(); ii++)
			{
			if( (lPedInView[ii].ped_pose.x-robotx_) > 0 && (lPedInView[ii].ped_pose.x-robotx_) < 5 )
				{
					   safeAction = 0;
				}
			}

    }
    else//Wait for a certain amount of gap in order to merge!
    {
		if(lPedInView.size()==0)
		{
			momdp_speed_ = 3;
			return;
		}
		safeAction=1;
			for(int ii=0; ii<lPedInView.size(); ii++)
			{
			if( lPedInView[ii].ped_pose.x < 5 &&  lPedInView[ii].ped_pose.x > -10 )
				{
					   safeAction = 0;
					   volkan.linear.x = lPedInView[ii].ped_pose.x;
					   volkan.linear.y = roboty_;
					   RobState_pub.publish(volkan);
				}
			}
    }

    if(!stationary_)
    {
        /// TBP : Change numbers to parameters
        if(safeAction==0) momdp_speed_ = 0;
        else if(safeAction==1) momdp_speed_ = 3;

    }
    clean_momdp_problem();

    for(int ii=0; ii<lPedInView.size(); ii++)
    {
        int id = lPedInView[ii].id;
        updateBelief(id,safeAction);
    }



}////////////////////////////////////////////////////////////////////////////////////////////








void ped_momdp::clean_momdp_problem()
{
    for( int ii=0; ii< lPedInView.size(); ii++)
    {
        int ped_x = getXGrid(lPedInView[ii].ped_pose.x);
        int ped_y = getYGrid(lPedInView[ii].ped_pose.y);
  //V      int rob_y = getYGrid(lPedInView[ii].rob_pose);
        ///erase momdp problem when ped_x!=1 && ped_y == rob_y
        
        ros::Duration time_update = ros::Time::now() - lPedInView[ii].last_update;
        cout<<"Time update for ped id is "<<lPedInView[ii].id<< ": "<<time_update.toSec()<<endl;
        
        if(ped_x <-15  ||  ped_x >16 || ped_y <0 )
       {
        cout<<"Erase ped id "<<lPedInView[ii].id<<" due to ped now fall outside of the sensor range."<<endl;
        lPedInView.erase(lPedInView.begin()+ii);
       }

        else if( time_update > ros::Duration(momdp_problem_timeout))
        {
            cout<<"Erase ped id "<<lPedInView[ii].id<<" due to timeout."<<endl;
            lPedInView.erase(lPedInView.begin()+ii);
        }

        
    }
}

////////////////////V No Need For This Function. THis publish messages are just for analysis, no one uses this messages!////////////////////////

 void ped_momdp::publish_belief()
{

    ped_momdp_sarsop::peds_believes peds_believes;
    for( int ii=0; ii< lPedInView.size(); ii++)
    {
        ped_momdp_sarsop::ped_belief ped_belief;

        /// Publish ped id
        ped_belief.ped_id = lPedInView[ii].id;
        ped_belief.ped_x = getXGrid(lPedInView[ii].ped_pose.x);
        ped_belief.ped_y = getYGrid(lPedInView[ii].ped_pose.y);

        /// Publish rob
        ped_belief.rob_x = getXGrid(lPedInView[ii].rob_pose.x);
        ped_belief.rob_y = getYGrid(lPedInView[ii].rob_pose.y);
        ///Publish belief
        int belief_size = lPedInView[ii].currBelSt->bvec->data.size();

        assert(belief_size<=4);

        ped_belief.belief_value.resize(4);
		
		total_belief_value = 0;
        for (int jj=0; jj < belief_size; jj++)
        {
            int belief_id = lPedInView[ii].currBelSt->bvec->data[jj].index;
            double belief_value = lPedInView[ii].currBelSt->bvec->data[jj].value;
            total_belief_value = total_belief_value + belief_value; 
            ped_belief.belief_value[belief_id] = belief_value;

        }


        ///Publish actions
        int temp_action;
        if(lPedInView[ii].currAction==2) temp_action = -1;
        else temp_action = lPedInView[ii].currAction;
        ped_belief.action = temp_action;

        peds_believes.believes.push_back(ped_belief);
        
		/////Belief Values Disappeared Check////////
		if(total_belief_value == 0)
		analysisvar.data = 3;
		///////////////////////////////////////////
    }
    peds_believes.cmd_vel = momdp_speed_;
    peds_believes.robotv = robotspeedx_;

    believesPub_.publish(peds_believes);
    
}

int ped_momdp::policy_initialize(string model_file, string policy_file, int simLen, int simNum)
{
    solver_param = &GlobalResource::getInstance()->solverParams;

    solver_param->problemName = model_file;
    solver_param->policyFile = policy_file;
    solver_param->simLen = simLen;
    solver_param->simNum = simNum;
    //check validity of options
    if (solver_param->policyFile == "" || solver_param->simLen == -1 || solver_param->simNum == -1)
    {
        //print_usage(p->cmdName);
        ROS_WARN("policy_initialize: solver_param options not valid");
        return 0;
    }

    cout << "\nLoading the model ..." << endl << "  ";
    problem = ParserSelector::loadProblem(solver_param->problemName, *solver_param);

    if(solver_param->stateMapFile.length() > 0 )
    {
        // generate unobserved state to variable value map
        ofstream mapFile(solver_param->stateMapFile.c_str());
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
    cout << "  input file   : " << solver_param->policyFile << endl;
    bool policyRead = policy->readFromFile(solver_param->policyFile);
    if(!policyRead)
    {
        ROS_WARN("Cannot read policy file");
        return 0;
    }


    cout << "\nSimulating ..." << endl;

    if(solver_param->useLookahead) /// TBP : always using one step look ahead
    {
        cout << "  action selection :  one-step look ahead" << endl;
    }

    srand(solver_param->seed);
    /// Initialize MOMDP Engine
    engine.setup(problem, policy, solver_param); /// TBP : p is params

    double reward = 0, expReward = 0;

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

    return 1;

}

void ped_momdp::updateBelief(int id, int safeAction)
{
    
    //cout << "Next state " << endl;

    int ii=-1;
    for(int jj=0; jj<lPedInView.size(); jj++)
    {
        if(lPedInView[jj].id == id)
        {
            ii = jj;
            cout << "lPedInView list position " << ii << endl;
            break;

        }
    }
	cout << "---- subproblem update  #" << id << "  safeAction " << safeAction << " ---- " << "  lastupdate "<<lPedInView[ii].last_update<<endl;
    if(ii==-1)
    {
        cout << "Cannot do belief update ids dont match ... " << endl;
        return;
    }


    //int nextSVal = getCurrentState(robotspeedx_, lPedInView[ii].rob_pose, lPedInView[ii].ped_pose.x, lPedInView[ii].ped_pose.y);

    /// Debug
    int nextSVal = getCurrentState(lPedInView[ii].rob_pose.x, lPedInView[ii].rob_pose.y, lPedInView[ii].ped_pose.x, lPedInView[ii].ped_pose.y);
    int currObservation = getCurrObs(ii);

    cout << "before update belief" << endl;
    (lPedInView[ii].currBelSt)->bvec->write(cout); cout << endl;
    cout << "curr bel sval " << (lPedInView[ii].currBelSt)->sval << endl;

    SharedPointer<BeliefWithState> nextBelSt;
    engine.runStep((lPedInView[ii].currBelSt), safeAction, currObservation, nextSVal, nextBelSt );

    copy(*(lPedInView[ii].currBelSt)->bvec, *nextBelSt->bvec);
    (lPedInView[ii].currBelSt)->sval = nextSVal;

    cout << "next belief" << endl;
    (lPedInView[ii].currBelSt)->bvec->write(cout); cout << endl;
    cout << "next bel sval " << (lPedInView[ii].currBelSt)->sval << endl;

    //map<string, string> aa = problem->getActionsSymbols(safeAction);
    //cout << "safe action " << aa["action_robot"] << endl;

    ROS_INFO ("next bel sval %d", (lPedInView[ii].currBelSt)->sval);
}



int ped_momdp::getCurrentState(double robx, double roby, double pedx, double pedy)
{

	int StateVal;

	if (pedx<0) //Rounding strategy is different for negative and positive numbers!
	pedx=pedx-dX;
	int px = getXGrid(pedx);
	int py = getYGrid(pedy);
	char ped_str[30];
	if(px>-15 && px<0)
	{
	sprintf(ped_str,"sD%d",px+14);
	volkan.angular.z = px+14;
	}
	else if(py>=5 && px>=0 && px<3)
	{
	sprintf(ped_str,"sD%d",px+14);
	volkan.angular.z = px+14;
	}
	else if(px>=3 && px<16)
	{
	sprintf(ped_str,"sD%d",px+14);
	volkan.angular.z = px+14;
	}
	else if(py>=0 && py<5)
	{
	sprintf(ped_str,"sD%d",34-py);
	volkan.angular.z = 34-py;
	}
	else
	{
	sprintf(ped_str,"sD35");
	volkan.angular.z = 35;
	}
    RobState_pub.publish(volkan);//for state and observation message publish

	int rx = getXGrid(robx);
	int ry = getYGrid(roby);
	char rob_str[30];
	if(rx<2 && ry>=0 && ry<5)
	{
	sprintf(rob_str,"sR%d",ry);
    volkan.angular.x = ry;
	}
	else if(ry==5 && rx>=0 && rx<2)
	{
	sprintf(rob_str,"sR5");
	volkan.angular.x = 5;
	}
	else if(rx>=2 && rx<16)
	{
	sprintf(rob_str,"sR%d",rx+4);
	volkan.angular.x = rx+4;
	}
	else
	{
	sprintf(rob_str,"sR20");
	volkan.angular.x = 20;
	}

    if(ry>Y_SIZE-1)
    {
        ROS_WARN("Robot went outside of the grid size, pushing the robot pose back to the edge");
        ry = Y_SIZE-1;
    }
    ///////////////////Publish Analysis DATA///////////////
    ///////////////Crash Or MergedControl/////////////
    if(volkan.angular.x >= 5)
    {
		if(volkan.angular.x==8)
		{
			analysisvar.data = 2;
		}
		else if(volkan.angular.z == volkan.angular.x + 10)
		{
			analysisvar.data = 1;
		}
	}
    AnalysisCrash_pub.publish(analysisvar);
    ros::Time time = ros::Time::now();
    double time_second = time.toSec();
    std_msgs::Float32 analysisvar2;
    analysisvar2.data = time_second;
    AnalysisTime_pub.publish(analysisvar2);
	/////////////////////////////////////////////////////////

    string state_str;
    state_str.append(ped_str);
    state_str.append(rob_str);



    StateVal = ObsStateMapping[state_str];

    cout << " Curr state " << state_str << " id " << StateVal << endl;

    return StateVal;
}

int ped_momdp::getCurrObs(int id)
{
    int ObsVal;
    //Perfect Observation (Same values of real states)

	if (lPedInView[id].ped_pose.x<0) //Rounding strategy is different for negative and positive numbers!
		lPedInView[id].ped_pose.x=lPedInView[id].ped_pose.x-dX;


    int px = getXGrid(lPedInView[id].ped_pose.x);
    int py = getYGrid(lPedInView[id].ped_pose.y);
    char ped_str[10];
    if(px>-15 && px<0)
    {
        sprintf(ped_str,"osD%d",px+14);
    	volkan.angular.y = px+14;

    }

    else if(py>=5 && px>=0 && px<3)
    {
        sprintf(ped_str,"osD%d",px+14);
    	volkan.angular.y = px+14;
    }

    else if(px>=3 && px<16)
    {
        sprintf(ped_str,"osD%d",px+14);
    	volkan.angular.y = px+14;
    }
    else if(py>=0 && py<5)
    {
        sprintf(ped_str,"osD%d",34-py);
    	volkan.angular.y = 34-py;
    }
    else
    {
        sprintf(ped_str,"osD35");
    	volkan.angular.y = 35;
    }

    RobState_pub.publish(volkan);//for state and observation message publish


    ObsVal = ObsSymbolMapping[ped_str];

    cout << "curr observation is " << ped_str << " id " << ObsVal << endl;


    return ObsVal;
}

int ped_momdp::getXGrid(double x)
{
    int px = (int) (x)/dX ;
    return px;
}

int ped_momdp::getYGrid(double y)
{
    int py = (int) (y)/dY ;
    return py;
}

