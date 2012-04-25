#include "momdp.h"

ped_momdp::ped_momdp(string model_file, string policy_file, int simLen, int simNum, bool stationary, double frequency, bool use_sim_time, ros::NodeHandle& nh)
{
    X_SIZE=4;
    Y_SIZE=10;
    dY= 2;// step size in Y
    dX= 2;// step size in X
    momdp_problem_timeout = 5.0;
    stationary_ = stationary;
    use_sim_time_ = use_sim_time;
    believesPub_ = nh.advertise<ped_momdp_sarsop::peds_believes>("peds_believes",1);
    cmdPub_ = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1);
    timer_ = nh.createTimer(ros::Duration(1.0/frequency), &ped_momdp::controlLoop, this);
    initPedGoal();
    policy_initialize(model_file, policy_file, simLen, simNum);
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
    bool foundPed=false;
    for(int jj=0; jj< lPedInView.size(); jj++)
    {

        if(lPedInView[jj].id==ped.ped_id)
        {
            //given in ROS coordinate, convert to momdp compatible format
            lPedInView[jj].ped_pose.x = -ped.ped_pose.y;
            lPedInView[jj].ped_pose.y = ped.ped_pose.x;
            lPedInView[jj].rob_pose = ped.rob_pose.x;
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

    if(speed.linear.x < momdp_speed_) cmd.linear.x = speed.linear.x;
    else cmd.linear.x = momdp_speed_;

    geometry_msgs::Twist cmd_temp;
    cmd_temp = cmd;

    cmdPub_.publish(cmd_temp);
}

void ped_momdp::initPedGoal()
{
    lPedGoal.clear();

    GOAL G0 = { 0, 0 };
    lPedGoal.push_back(G0);

    GOAL G1 = { X_SIZE-1, 0 };
    lPedGoal.push_back(G1);

    GOAL G2 = { X_SIZE-1, Y_SIZE-1 };
    lPedGoal.push_back(G2);

    GOAL G3 = { 0, Y_SIZE-1 };
    lPedGoal.push_back(G3);

    return;
}
void ped_momdp::initPedMOMDP(ped_momdp_sarsop::ped_local_frame& ped_local)
{
    PED_MOMDP pedProblem;
    pedProblem.id = ped_local.ped_id;

    cout<<"create new belief state"<<endl;
    pedProblem.currBelSt = (new BeliefWithState());

    //cout<<"Ped pose: "<<ped_local.ped_pose<<endl;

    pedProblem.currSVal = getCurrentState(robotspeedx_, ped_local.rob_pose.x, -ped_local.ped_pose.y, ped_local.ped_pose.x);
    /// Debug
    //pedProblem.currSVal = getCurrentState(0, 0, -ped_local.ped_pose.y, ped_local.ped_pose.x);
    cout << "Current SVal " << pedProblem.currSVal << endl;

    SharedPointer<SparseVector> startBeliefVec;
    if (problem->initialBeliefStval->bvec)
        startBeliefVec = problem->initialBeliefStval->bvec;
    else
        startBeliefVec = problem->initialBeliefYByX[pedProblem.currSVal];


    ///// initializing belief for Y
    //int currUnobsState = chooseFromDistribution(*startBeliefVec);
    //int belSize = startBeliefVec->size();


    pedProblem.currBelSt->sval = pedProblem.currSVal;
    copy(*(pedProblem.currBelSt)->bvec, *startBeliefVec);

    //cout << "Starting Belief " << endl;
    //pedProblem.currBelSt->bvec->write(cout);
    //cout << endl;


    pedProblem.currAction = policy->getBestActionLookAhead(*(pedProblem.currBelSt));

    //Add newly observed pedestrian
    pedProblem.rob_pose = (double)ped_local.rob_pose.x;
    pedProblem.ped_pose.x = -(double)ped_local.ped_pose.y;
    pedProblem.ped_pose.y = (double)ped_local.ped_pose.x;
    lPedInView.push_back(pedProblem);
}
void ped_momdp::controlLoop(const ros::TimerEvent &e)
{

    cout << "Control loop dj .. " << " lPedInView " << lPedInView.size() << endl;

    //publish goal point at 25 meter in global frame
    /*geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "/map";
    ps.pose.position.x = 3;
    ps.pose.position.y = 25;
    ps.pose.orientation.w = 1.0;
    goalPub_.publish(ps);*/

    if(lPedInView.size()==0)
    {
        momdp_speed_ = 1.5;
        return;
    }

    ///Start pomdp stuff
    cout << "=====================================================================" << endl;

    /// Get new action
    for(int ii=0; ii<lPedInView.size(); ii++)
    {
        lPedInView[ii].currAction = policy->getBestActionLookAhead(*(lPedInView[ii].currBelSt));
    }

    /// combining the actions by picking the safest action
    /// Do the far away pedestrians create a freezing action ???

    int safeAction=1; /// actions : cru=0, acc=1, decc=2
    for(int ii=0; ii<lPedInView.size(); ii++)
    {
        if( (lPedInView[ii].currAction!=safeAction) && (safeAction!=2) )
        {
            if(lPedInView[ii].currAction==2)
                safeAction = 2;
            else if (lPedInView[ii].currAction==0)
                safeAction =0;
        }
    }

    map<string, string> aa = problem->getActionsSymbols(safeAction);
    cout << "safe action " << aa["action_robot"] << endl;


    if(!stationary_)
    {
        /// TBP : Change numbers to parameters
        if(safeAction==0) momdp_speed_ += 0;
        else if(safeAction==1) momdp_speed_ += 0.5;
        else if(safeAction==2) momdp_speed_ -= 0.5;
        if(momdp_speed_<=0) momdp_speed_ = 0;
        if(momdp_speed_>=1.5) momdp_speed_ = 1.5;
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

void ped_momdp::clean_momdp_problem()
{
    for( int ii=0; ii< lPedInView.size(); ii++)
    {
        int ped_x = getXGrid(lPedInView[ii].ped_pose.x);
        int ped_y = getYGrid(lPedInView[ii].ped_pose.y);
        int rob_y = getYGrid(lPedInView[ii].rob_pose);
        ///erase momdp problem when ped_x!=1 && ped_y == rob_y
        
        ros::Duration time_update = ros::Time::now() - lPedInView[ii].last_update;
        cout<<"Time update for ped id is "<<lPedInView[ii].id<< ": "<<time_update.toSec()<<endl;
        
        if(ped_x !=1 && (ped_y == rob_y))
        {
            cout<<"Erase ped id "<<lPedInView[ii].id<<" due to ped now fall outside of the FOV."<<endl;
            lPedInView.erase(lPedInView.begin()+ii);
        }
        else if( time_update > ros::Duration(momdp_problem_timeout))
        {
            cout<<"Erase ped id "<<lPedInView[ii].id<<" due to timeout."<<endl;
            lPedInView.erase(lPedInView.begin()+ii);
        }

        
    }
}


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
        ped_belief.rob_x = 1;//getXGrid(lPedInView[ii].rob_pose.x);
        ped_belief.rob_y = getYGrid(lPedInView[ii].rob_pose);
        ///Publish belief
        int belief_size = lPedInView[ii].currBelSt->bvec->data.size();

        assert(belief_size<=4);

        ped_belief.belief_value.resize(4);

        for (int jj=0; jj < belief_size; jj++)
        {
            int belief_id = lPedInView[ii].currBelSt->bvec->data[jj].index;
            double belief_value = lPedInView[ii].currBelSt->bvec->data[jj].value;

            ped_belief.belief_value[belief_id] = belief_value;

        }


        ///Publish actions
        int temp_action;
        if(lPedInView[ii].currAction==2) temp_action = -1;
        else temp_action = lPedInView[ii].currAction;
        ped_belief.action = temp_action;

        peds_believes.believes.push_back(ped_belief);
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
    int nextSVal = getCurrentState(robotspeedx_,  lPedInView[ii].rob_pose, lPedInView[ii].ped_pose.x, lPedInView[ii].ped_pose.y);
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

int ped_momdp::getCurrentState(double currRobSpeed, double roby, double pedx, double pedy)
{
    int StateVal;

    int px = getXGrid(pedx);
    int py = getYGrid(pedy);
    if(py==Y_SIZE-1) py=Y_SIZE-2;
    char ped_str[30];
    sprintf(ped_str,"sx%02dy%02d",px,py);


    int ry = getYGrid(roby);
    if(ry==Y_SIZE-1) ry=Y_SIZE-2;
    char rob_str[30];
    sprintf(rob_str,"sR%02d",ry);

    //if(myDebug)
    //sprintf(rob_str,"sR%02d",currRobY);

    //cout<<"rob_str"<<endl;
    if(ry>Y_SIZE-1)
    {
        ROS_WARN("Robot went outside of the grid size, pushing the robot pose back to the edge");
        ry = Y_SIZE-1;
    }


    //dj: discretization of robot speed into 3 int levels (0,1,2)
    double rvel_double;
    if(currRobSpeed < 0.1) rvel_double = 0;
    else if(currRobSpeed > 0.1 && currRobSpeed < 2) rvel_double = 1;
    else if(currRobSpeed > 2) rvel_double = 2;
    //double rvel_double =  currRobSpeed/1.5*2;//dSpeed;
    int rvel= (int) rvel_double;
    //if(rvel_double < 0.0001 )

    char rob_vel_str[30];
    sprintf(rob_vel_str,"sV%d",rvel);

    cout << " rvel : " << rob_vel_str << endl;
    //if(myDebug)
    //sprintf(rob_vel_str,"sV%d",currRobVel);

    string state_str;
    state_str.append(ped_str);
    state_str.append(rob_str);
    state_str.append(rob_vel_str);


    StateVal = ObsStateMapping[state_str];

    cout << " Curr state " << state_str << " id " << StateVal << endl;

    return StateVal;
}

int ped_momdp::getCurrObs(int id)
{
    int ObsVal;
    /// Poll sensors
    /// get ped location

    int px = getXGrid(lPedInView[id].ped_pose.x);
    int py = getYGrid(lPedInView[id].ped_pose.y);
    char ped_str[10];
    if(py == Y_SIZE-1) py=Y_SIZE-2;
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

int ped_momdp::getXGrid(double x)
{
    int px = (int) (x)/dX ;
    if(px> (X_SIZE-1)) px = (X_SIZE-1);
    else if (px<0) px = 0;
    return px;
}

int ped_momdp::getYGrid(double y)
{
    int py = (int) y/dY ;
    if(py> (Y_SIZE-1)) py = (Y_SIZE-1);
    else if (py<0) py = 0;

    return py;
}
