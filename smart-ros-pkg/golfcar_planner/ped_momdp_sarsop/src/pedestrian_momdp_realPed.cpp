/*
 * pedestrian_momdp.cpp
 *
 *  Created on: Sep 15, 2011
 *      Author: golfcar
 */

#include "pedestrian_momdp_realPed.h"
using namespace std;



void pedestrian_momdp::initPedGoal()
{

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


void pedestrian_momdp::parse_simul_config( fstream& configfile )
{
    ///// simulation variables are ped pos, ped uncert

    ////char input[50]; /// collect input params, white space separator

    //lane = false;
    //bool crosssmall = false;
    //bool crosslarge = false;
    //ell = false;

    //int env, ulevel;

    //string input;

    //cout << " Parsing input " ;
    //while(!configfile.eof())
    //{
    //configfile >> input;

    ////cout << " Parsing input " << input << endl;

    //if(input.find("env:") != string::npos)
    //{
    //sscanf(input.c_str(), " env:%d ", &env);
    //cout << " env:" ;

    //if(env==0)
    //{
    //lane=true;
    //cout << " lane " ;
    //}
    //else if( env==1 )
    //{
    //crosssmall=true;
    //cout << " crosssmall " ;
    //cross=true;
    //}
    //else if ( env==2)
    //{
    //crosslarge=true;
    //cout << " crosslarge " ;
    //cross=true;
    //}
    //else if ( env==3)
    //{
    //ell = true;
    //cout << " ell ";
    //}
    //}
    //else if(input.find("algo:") != string::npos)
    //{
    //sscanf(input.c_str(), " algo:%d ", &mostLikelyAlgo);
    //cout << " algo: " << mostLikelyAlgo;
    //}
    //else if(input.find("X:") != string::npos)
    //{
    //sscanf(input.c_str(), " X:%d ", &X_SIZE);
    //cout << " X: " << X_SIZE;
    //}
    //else if(input.find("Y:") != string::npos)
    //{
    //sscanf(input.c_str(), " Y:%d ", &Y_SIZE);
    //cout << " Y: " << Y_SIZE;
    //}
    //else if(input.find("robx:") != string::npos)
    //{
    //sscanf(input.c_str(), " robx:%d ", &robx);
    //cout << " robx: " << robx;
    //}
    ////else if(input.find("Y_INTER:") != string::npos)
    ////{
    ////sscanf(input.c_str(), " Y_INTER:%d ", &Y_INTER);
    ////}
    ////else if(input.find("simul:") != string::npos)
    ////{
    ////sscanf(input.c_str(), " simul:%d ", &simul);
    ////}
    //else if(input.find("pedx:") != string::npos)
    //{
    //sscanf(input.c_str(), " pedx:%d ", &pedx);
    //cout << " pedx: " << pedx;
    //}
    //else if(input.find("pedy:") != string::npos)
    //{
    //sscanf(input.c_str(), " pedy:%d ", &pedy);
    //cout << " pedy: " << pedy;
    //}
    //else if(input.find("pedU:") != string::npos)
    //{
    //sscanf(input.c_str(), " pedU:%d ", &ulevel);
    //cout << " pedU: " << ulevel;
    //}
    //else if(input.find("lx:") != string::npos)
    //{
    //sscanf(input.c_str(), " lx:%d ", &l_x);
    //cout << " lx: " << l_x;
    //}
    //else if(input.find("ly:") != string::npos)
    //{
    //sscanf(input.c_str(), " ly:%d ", &l_y);
    //cout << " ly: " << l_y;
    //}
    //else if(input.find("rx:") != string::npos)
    //{
    //sscanf(input.c_str(), " rx:%d ", &r_x);
    //cout << " rx: " << r_x;
    //}
    //else if(input.find("ry:") != string::npos)
    //{
    //sscanf(input.c_str(), " ry:%d ", &r_y);
    //cout << " ry: " << r_y;
    //}
    //else
    //{
    //cout << endl << " unused input " << input << "  " << endl;
    //}
    //} /// end parsing

    //cout << endl << " out of parsing " << endl;

    ///// Player stage mapping
    //if(lane)
    //{
    //PSG_Y=60;
    //PSG_X=14;
    ////X_SIZE=4;
    ////Y_SIZE=10;
    ////NUM_GOAL=4;
    //}
    //else //if(cross)
    //{
    //PSG_X=44.0;
    //PSG_Y=64.0;
    ////X_SIZE=12;
    ////Y_SIZE=17;
    ////NUM_GOAL=6;
    //}

    //char type[10];
    //char name[100];
    //if(mostLikelyAlgo)
    //sprintf(type,"ML");
    //else
    //sprintf(type,"momdp");

    //cout << endl << " Problem " << env << " pedx:" << pedx << " pedy:" << pedy << " robx:" << robx << endl << endl;

    //if(lane)
    //{
    //sprintf(name,"stats-lane-%s-x%02d-y%02d-r%02d-u%02d.dat",type,pedx,pedy,robx,ulevel);
    ////rx_i = 1;
    //}
    //else if(crosssmall)
    //{
    //sprintf(name,"stats-cross-small-%s-x%02d-y%02d-r%02d-u%02d.dat",type,pedx,pedy,robx,ulevel);
    ////rx_i = 5;
    //}
    //else if(crosslarge)
    //{
    //sprintf(name,"stats-cross-large-%s-x%02d-y%02d-r%02d-u%02d.dat",type,pedx,pedy,robx,ulevel);
    ////rx_i = 5;
    //}
    //else if(ell)
    //{
    //sprintf(name,"stats-ell-%s-x%02d-y%02d-r%02d-u%02d.dat",type,pedx,pedy,robx,ulevel);
    ////sprintf(name,"stats-ell-%s.dat",type);
    //}
    //else
    //{
    //cout << " unknown environment " << endl;
    //exit(1);
    //}
    //statfile.open (name);

}

int pedestrian_momdp::pomdp_initialize()
{
    p = &GlobalResource::getInstance()->solverParams;

    /*bool parseCorrect = SolverParams::parseCommandLineOption(argc, argv, *p);
    if(!parseCorrect)
    {
        //print_usage(p->cmdName);
        exit(EXIT_FAILURE);
    }*/
    p->problemName = model_file;
    p->policyFile = policy_file;
    p->simLen = simLen;
    p->simNum = simNum;
    //check validity of options
    if (p->policyFile == "" || p->simLen == -1 || p->simNum == -1)
    {
        //print_usage(p->cmdName);
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

    //SimulationRewardCollector rewardCollector;
    //rewardCollector.setup(*p);

    //ofstream * foutStream = NULL;
    srand(p->seed);//Seed for random number.  Xan
    //cout << p->seed << endl;




    if (enableFiling)
    {
        foutStream = new ofstream(p->outputFile.c_str());
    }




    /// Initialize MOMDP Engine
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

    //for(int ii=0; ii<num_ped; ii++)
    //currSVal[ii] = getCurrentState(ii);

    int currSVal[num_ped];
    vector< SharedPointer<BeliefWithState> > lcurrBelSt;
    int currAction[num_ped];

    for(int ii=0; ii<num_ped; ii++)
    {
        cout<<"Final loop: "<<ii<<endl;

        SharedPointer<BeliefWithState> currBelSt (new BeliefWithState());

        currSVal[ii] = getCurrentState(ii);

        SharedPointer<SparseVector> startBeliefVec;
        if (problem->initialBeliefStval->bvec)
            startBeliefVec = problem->initialBeliefStval->bvec;
        else
            startBeliefVec = problem->initialBeliefYByX[currSVal[ii]];
        cout<<"After belief started"<<endl;

        /// TBP : initializing belief for Y
        int currUnobsState = chooseFromDistribution(*startBeliefVec);
        int belSize = startBeliefVec->size();


        currBelSt->sval = currSVal[ii];
        copy(*currBelSt->bvec, *startBeliefVec);
        cout << "Starting Belief " << endl;
        cout<<"Just before write"<<endl;
        currBelSt->bvec->write(cout);//, *streamOut);
        cout << endl;

        lcurrBelSt.push_back(currBelSt);
        cout<<"Get best action"<<endl;
        currAction[ii] = policy->getBestActionLookAhead(*currBelSt);
    }
    double mult=1;
    double gamma = problem->getDiscount();



}

void pedestrian_momdp::publish_belief()
{

    ped_momdp_sarsop::peds_believes peds_believes;
    for( int ii=0; ii< num_ped; ii++)
    {
        ped_momdp_sarsop::ped_belief ped_belief;
        /// Publish ped id
        ped_belief.ped_id = lPedInView[ii].id;
        ped_belief.x = getXGrid(lPedInView[ii].pedx_);
        ped_belief.y = getYGrid(lPedInView[ii].pedy_);
        ///Publish belief
        int belief_size = lcurrBelSt[ii]->bvec->data.size();
        assert(belief_size<=4);
        ped_belief.belief_value.resize(4);
        for (int jj=0; jj < belief_size; jj++)
        {
            int belief_id = lcurrBelSt[ii]->bvec->data[jj].index;
            double belief_value = lcurrBelSt[ii]->bvec->data[jj].value;

            ped_belief.belief_value[belief_id] = belief_value;
        }
        
        
        ///Publish actions
        int temp_action;
        if(currAction[ii]==2) temp_action = -1;
        else temp_action = currAction[ii];
        ped_belief.action = temp_action;

        peds_believes.believes.push_back(ped_belief);
    }
    peds_believes.cmd_vel = cmd.linear.x;
    peds_believes.robotv = robotspeedx_;
    peds_believes.robotx = getXGrid(robotx_);
    peds_believes.roboty = getYGrid(roboty_);
    believesPub_.publish(peds_believes);
}

void pedestrian_momdp::pedInitPose()
{
    cout<<"Initialize pose "<<ped_id_file<<endl;
    fstream pedinitfile;
    pedinitfile.open(ped_id_file.c_str());

    for (int ii=0; ii< num_ped; ii++)
    {
        PED ped1;
        pedinitfile >> ped1.id;
        cout<<"id "<<ii<<": "<<ped1.id<<endl;
        //pedinitfile >> ped1.pedx_;
        //pedinitfile >> ped1.pedy_;

        lPedInView.push_back(ped1);
    }

    pedinitfile.close();
}

pedestrian_momdp::pedestrian_momdp(int argc, char** argv) {
    ros::NodeHandle nh;
    robotSub_ = nh.subscribe("robot_0/amcl_pose", 1, &pedestrian_momdp::robotPoseCallback, this);
    speedSub_ = nh.subscribe("odom", 1, &pedestrian_momdp::speedCallback, this);
    pedSub_ = nh.subscribe("ped_map_laser_batch", 1, &pedestrian_momdp::pedPoseCallback, this);
    ros::NodeHandle n("~");
    /// subscribe to
    n.param("pedestrian_id_file", ped_id_file, string(""));
    n.param("policy_file", policy_file, string(""));
    n.param("model_file", model_file, string(""));
    n.param("simLen", simLen, 100);
    n.param("simNum", simNum, 100);
    nh.param("use_sim_time", use_sim_time_, false);
    //add the simLen parameter and correct the coordinate system!

    //subscribe as heartbeat of the robot
    //scanSub_ = nh.subscribe("clock", 1, &pedestrian_momdp::scanCallback, this);
    cmdPub_ = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1);
    believesPub_ = nh.advertise<ped_momdp_sarsop::peds_believes>("peds_believes",1);
    move_base_speed_=nh.subscribe("/move_status",1, &pedestrian_momdp::moveSpeedCallback, this);
    goalPub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
    initPedGoal();
    //pedInitPose();

    /////pomdp initialize configuration
    //fstream  variablefile;
    //variablefile.open("config-simul-rss-psg.txt");
    //assert (!variablefile.fail( ));
    //cout << " done opening " << endl;
    //parse_simul_config(variablefile);
    //cout << " done parsing " << endl;



    timer_ = nh.createTimer(ros::Duration(1.0), &pedestrian_momdp::controlLoop, this);
    ros::spin();
}

pedestrian_momdp::~pedestrian_momdp()
{
    cmd.angular.z = 0;
    cmd.linear.x = 0;
    cmdPub_.publish(cmd);
}

void pedestrian_momdp::speedCallback(nav_msgs::Odometry odo)
{
    robotspeedx_ = odo.twist.twist.linear.x;
}

void pedestrian_momdp::robotPoseCallback(geometry_msgs::PoseWithCovarianceStamped odo)
{
    robotx_ = odo.pose.pose.position.x;
    roboty_ = odo.pose.pose.position.y;


    if(robotx_>0 && roboty_>0) robot_pose = true;
}

void pedestrian_momdp::moveSpeedCallback(pnc_msgs::move_status status)
{
    cmd.angular.z = status.steer_angle;
    if(roboty_>20)
    {
        cmd.angular.z = 0;
        cmd.linear.x = 0;
    }

    geometry_msgs::Twist cmd_temp;
    cmd_temp = cmd;
    if(use_sim_time_) cmd_temp.linear.x = cmd.linear.x * 0.3;
    cmdPub_.publish(cmd_temp);
}

void pedestrian_momdp::pedPoseCallback(sensing_on_road::pedestrian_laser_batch laser_batch)
{
    /// change code TBP
    //pedx_ = -pc.points[0].y;
    //pedy_ = pc.points[0].x;

    //direct initialization since it is taken care by transformPedtoMap
    if(!obs_flag)
    {
        num_ped = laser_batch.pedestrian_laser_features.size();
        lPedInView.resize(num_ped);
        currSVal.resize(num_ped);
        currAction.resize(num_ped);
    }
    ROS_DEBUG_STREAM("Laser batch size "<<laser_batch.pedestrian_laser_features.size()<<" lPedInView size "<<lPedInView.size());
    for(int ii=0; ii< laser_batch.pedestrian_laser_features.size(); ii++)
    {
        lPedInView[ii].pedx_ = laser_batch.pedestrian_laser_features[ii].pedestrian_laser.point.x;
        lPedInView[ii].pedy_ = laser_batch.pedestrian_laser_features[ii].pedestrian_laser.point.y;
        lPedInView[ii].id = laser_batch.pedestrian_laser_features[ii].object_label;
        ROS_DEBUG_STREAM(ii<<": PedX "<<lPedInView[ii].pedx_ << " PedY "<<lPedInView[ii].pedy_);
        obs_flag = true;
    }



}

void pedestrian_momdp::controlLoop(const ros::TimerEvent &e)
{

    /// Start after 3 pedestrians are detected
    cout<<"Pedestrian: "<<obs_flag<<" Robot: "<<robot_pose<<endl;
    if(!obs_flag || !robot_pose)return;

    //publish goal point at 25 meter in global frame
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "/map";
    ps.pose.position.x = 3;
    ps.pose.position.y = 25;
    ps.pose.orientation.w = 1.0;
    goalPub_.publish(ps);

    if(obs_first)
    {

        pomdp_initialize();
        for(int ii=0; ii<num_ped; ii++)
        {

            SharedPointer<BeliefWithState> currBelSt (new BeliefWithState());

            currSVal[ii] = getCurrentState(ii);

            SharedPointer<SparseVector> startBeliefVec;
            cout<<"ControlLoop: startBelief"<<endl;
            if (problem->initialBeliefStval->bvec)
                startBeliefVec = problem->initialBeliefStval->bvec;
            else
                startBeliefVec = problem->initialBeliefYByX[currSVal[ii]];

            cout<<"ControlLoop: EndStartBelief"<<endl;
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
    //Start pomdp stuff
    cout << "=====================================================================" << endl;
    num_steps++;
    cout << "Curr State in loop : " << endl; //getCurrentState();/// Just to print current state


    //int currAction = policy->getBestActionLookAhead(*currBelSt);
    int mlcurrAction[num_ped];
    for(int ii=0; ii<num_ped; ii++)
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
    for(int ii=0; ii<num_ped; ii++)
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

        if(safeAction==0) cmd.linear.x += 0;
        else if(safeAction==1) cmd.linear.x += 0.5;
        else if(safeAction==2) cmd.linear.x -= 0.5;
        if(cmd.linear.x<=0) cmd.linear.x = 0;
        if(cmd.linear.x>=1.5) cmd.linear.x = 1.5;

    }


    publish_belief();


    //for(int ii=0; ii<num_ped; ii++)
    //gp[ii]->Clear();

    /// update belief
    for(int ii=0; ii<num_ped; ii++)
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
        //map<string, string> aa = problem->getActionsSymbols(currAction);
        map<string, string> aa = problem->getActionsSymbols(safeAction);
        cout << "safe action " << aa["action_robot"] << endl;

        ROS_INFO ("next bel sval %d", (lcurrBelSt[ii])->sval);
    }



    //for(int ii=0; ii<num_ped; ii++)
    //{
    //cout << "---- subproblem update  #" << ii << " ---- " << endl;
    //cout << "Next state " << endl;
    //int nextSVal = getCurrentState(ii);
    //int currObservation = getCurrObs(ii);

    ////double currReward = engine.getReward(*currBelSt, currAction);
    ////expReward += mult*currReward;
    ////mult *= gamma;
    ////reward += currReward;

    ////cout << "CurrReward " << currReward << " ExpReward " << expReward << endl;



    //cout << "before update belief" << endl;
    //(lcurrBelSt[ii])->bvec->write(cout); cout << endl;
    //cout << "curr bel sval " << (lcurrBelSt[ii])->sval << endl;

    //SharedPointer<BeliefWithState> nextBelSt;
    //engine.runStep((lcurrBelSt[ii]), safeAction, currObservation, nextSVal, nextBelSt );

    //copy(*(lcurrBelSt[ii])->bvec, *nextBelSt->bvec);
    //(lcurrBelSt[ii])->sval = nextSVal;

    //cout << "next belief" << endl;
    //(lcurrBelSt[ii])->bvec->write(cout); cout << endl;
    ////ROS_INFO << "next bel sval " << (lcurrBelSt[ii])->sval << endl;
    //ROS_INFO ("next bel sval %d", (lcurrBelSt[ii])->sval);

    //}
    //updateStats();
}

int pedestrian_momdp::getCurrObs(int id)//SharedPointer<MOMDP>& problem)
{
    int ObsVal;
    /// Poll sensors
    /// get ped location

    int px = getXGrid(lPedInView[id].pedx_);
    int py = getYGrid(lPedInView[id].pedy_);
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

int pedestrian_momdp::getCurrentState(int id)
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

    int px = getXGrid(lPedInView[id].pedx_);
    int py = getYGrid(lPedInView[id].pedy_);
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
    double rvel_double;
    if(currRobSpeed < 0.1) rvel_double = 0;
    else if(currRobSpeed > 0.1 && currRobSpeed < 2) rvel_double = 1;
    else if(currRobSpeed > 2) rvel_double = 2;
    //double rvel_double =  currRobSpeed/1.5*2;//dSpeed;
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

int pedestrian_momdp::getXGrid(double x)
{
    int px = (int) (x)/dX ;
    if(px> 3) px = 3;
    else if (px<0) px = 0;
    return px;
}

int pedestrian_momdp::getYGrid(double y)
{
    int py = (int) (y+Y_OFFSET)/dY ;
    return py;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mdp");

    pedestrian_momdp::pedestrian_momdp* mdp_node;
    mdp_node = new pedestrian_momdp::pedestrian_momdp(argc, argv);

    ros::spin();
}