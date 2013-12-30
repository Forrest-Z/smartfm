#include "momdp.h"

ped_momdp::ped_momdp(string model_file, string policy_file, int simLen, int simNum, bool stationary, double frequency, bool use_sim_time, ros::NodeHandle& nh)
{
	cout<<"momdp node start"<<endl;
    X_SIZE=4;
	//X_SIZE=ModelParams::XSIZE;
    Y_SIZE=ModelParams::YSIZE;
    dY= 1;// step size in Y
    dX= 1;// step size in X
    momdp_problem_timeout = 5.0;
    stationary_ = stationary;
    use_sim_time_ = use_sim_time;
    believesPub_ = nh.advertise<ped_momdp_sarsop::peds_believes>("peds_believes",1);
    cmdPub_ = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1);
  	timer_ = nh.createTimer(ros::Duration(1.0/frequency), &ped_momdp::controlLoop, this);
    //initPedGoal();
    //policy_initialize(model_file, policy_file, simLen, simNum);
	cout<<"here"<<endl;
	initRealSimulator();

}


void ped_momdp::initRealSimulator()
{
	RealSimulator=new PEDESTRIAN_CHANGELANE(ModelParams::XSIZE,ModelParams::YSIZE);
	RealSimulator->rob_map=RealWorldPt->window.rob_map;
	RealSimulator->sfm=&RealWorldPt->sfm;

	PedestrianState startState=RealWorldPt->GetCurrState();
	RealSimulator->SetStartState(startState);

    MCTS::InitFastUCB(50000);
    MCTS::PARAMS SearchParams;
	solver=new MCTS(*RealSimulator, SearchParams);
	solver->root_list.push_back(solver->Root);


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
	//cout<<"updatePedRobPose"<<endl;
    for(int jj=0; jj< lPedInView.size(); jj++)
    {

        if(lPedInView[jj].id==ped.ped_id)
        {
            //given in ROS coordinate, convert to momdp compatible format
			/*
            lPedInView[jj].ped_pose.x = -ped.ped_pose.y;
            lPedInView[jj].ped_pose.y = ped.ped_pose.x;
            lPedInView[jj].rob_pose = ped.rob_pose.x;*/
			
			//updated 12.6, now assume the input is already a correct position
			lPedInView[jj].ped_pose.x = ped.ped_pose.x;
			lPedInView[jj].ped_pose.y = ped.ped_pose.y;
            lPedInView[jj].rob_pose =   ped.rob_pose.y;
            //cout<<"rob pos "<<lPedInView[jj].rob_pose<<endl;
            ///// Debug
            foundPed=true;
			lPedInView[jj].last_update = ros::Time::now();
            ROS_DEBUG_STREAM( "Updated ped #" << ped.ped_id << " " <<lPedInView[jj].ped_pose.x<<' '<<lPedInView[jj].ped_pose.y);
			lPedInView[jj].obss_updated=true;
            break;

        }

    }
    return foundPed;
}

void ped_momdp::updateSteerAnglePublishSpeed(geometry_msgs::Twist speed)
{
	//cout<<"update steering angle!!!!!!!!!!!!!!"<<endl;
    geometry_msgs::Twist cmd;

    cmd.angular.z = speed.angular.z;
    double momdp_speed = momdp_speed_;
    //need correction here for the speed compensation for simulation
    //fixed ros-pkg ticket #5432
    //if(use_sim_time_) momdp_speed = momdp_speed * 0.3; /// TBP change numbers into parameters

    /*
    if(speed.linear.x < momdp_speed_) cmd.linear.x = speed.linear.x;
    //else 
    else cmd.linear.x = momdp_speed_;*/
    
    cmd.linear.x=momdp_speed_;
    //cout<<"momdp speed:"<<momdp_speed_<<endl;

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

	cout<<"ped_pose y"<<ped_local.ped_pose.y<<endl;
	cout<<"ped_pose x"<<ped_local.ped_pose.x<<endl;
    //pedProblem.currSVal = getCurrentState(robotspeedx_, ped_local.rob_pose.x, -ped_local.ped_pose.y, ped_local.ped_pose.x);
	pedProblem.currSVal = getCurrentState(robotspeedx_, ped_local.rob_pose.y, ped_local.ped_pose.x, ped_local.ped_pose.y,ped_local.ped_id);
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


    //pedProblem.currAction = policy->getBestActionLookAhead(*(pedProblem.currBelSt));
	pedProblem.currAction=2;

    //Add newly observed pedestrian
    pedProblem.rob_pose = (double)ped_local.rob_pose.y;
    pedProblem.ped_pose.x = (double)ped_local.ped_pose.x;
    pedProblem.ped_pose.y = (double)ped_local.ped_pose.y;
	pedProblem.obss_updated=true;
    lPedInView.push_back(pedProblem);
}

/*
int ped_momdp::reactivePolicy(int i)
{
	Car car=world.GetCarPos();
	int lane=car.w;
	cout<<"Using Policy "<<lane<<endl;
	if(lane<0||lane>4) return 0;
	//PEDESTRIAN_DYNAMIC_REAL_STATE * ped_state=safe_cast<PEDESTRIAN_DYNAMIC_REAL_STATE*>(state);
	int pedx=lPedInView[i].ped_pose.x;
	int pedy=lPedInView[i].ped_pose.y;
	int roby=lPedInView[i].rob_pose;
	int vel=robotspeedx_;
	int delta=pedy-roby;
	if(pedx>=lane-1&&pedx<=lane+1&&delta>0&&delta<=2)
	{
		return 2;	
	}
	if(delta<0)
	{
		return 1;
	}
	if(vel==0)
	{
		return 1;
	}
	if(vel==2&&delta>0)
	{
		return 2;
	}
	return 0;
}

void ped_momdp::initSimulator()
{
	lPedInView.clear();
	Car car=world.GetCarPos();
	cout<<"init num ped "<<world.NumPedInView()<<endl;
	robotspeedx_=world.velGlobal;
	for(int i=0;i<world.NumPedInView();i++)
	{
		Pedestrian ped=world.GetPedPose(i);	
		ped_momdp_sarsop::ped_local_frame plf;
		plf.ped_id=ped.id;
		plf.ped_pose.x=ped.w;
		plf.ped_pose.y=ped.h;
		plf.rob_pose.x=car.w;
		plf.rob_pose.y=car.h;
		cout<<ped.w<<" "<<ped.h<<endl;
		addNewPed(plf);
	}
}
void ped_momdp::updatePedPoses()
{
	//initSimulator();
	Car car=world.GetCarPos();
	robotspeedx_=world.velGlobal;
	for(int i=0;i<world.NumPedInView();i++)
	{
		Pedestrian ped=world.GetPedPose(i);	
		ped_momdp_sarsop::ped_local_frame plf;
		plf.ped_id=ped.id;
		plf.ped_pose.x=ped.w;
		plf.ped_pose.y=ped.h;
		plf.rob_pose.x=car.w;
		plf.rob_pose.y=car.h;
		bool foundPed=updatePedRobPose(plf);
		if(!foundPed)
		{
			addNewPed(plf);
		}
	}
}
void ped_momdp::updateObsStates()
{
	for(int i=0;i<lPedInView.size();i++)
	{
		(lPedInView[i].currBelSt)->sval=getCurrentState(robotspeedx_,  lPedInView[i].rob_pose, lPedInView[i].ped_pose.x, lPedInView[i].ped_pose.y,lPedInView[i].id);
	}
}

int ped_momdp::getActionMOMDP(int i)
{
	Car car=world.GetCarPos();
	int lane=car.w;
	cout<<"Using Policy "<<lane<<endl;
	if(lane<0||lane>4) return 0;
	//lPedInView[i].currAction = policies[lane]->getBestActionLookAhead(*(lPedInView[i].currBelSt));
	lPedInView[i].currAction = policies[lane]->getBestAction(*(lPedInView[i].currBelSt));
}
int ped_momdp::getActionQMDP(int i)
{
	Car car=world.GetCarPos();
	int lane=car.w;
	cout<<"Using Policy "<<lane<<endl;
	if(lane<0||lane>4) return 0;
	//lPedInView[i].currAction = policies[lane]->getBestActionLookAhead(*(lPedInView[i].currBelSt));
	return qmdp_policies[lane]->getBestAction(*(lPedInView[i].currBelSt));
}


int ped_momdp::getActionBML(int i)
{
	cout<<"BML"<<endl;
	SharedPointer<BeliefWithState> bmlBelSt=new BeliefWithState();
	int length=((lPedInView[i].currBelSt)->bvec->data).size();
	//nexBelSt->bvec->data.resize(length);
	copy(*bmlBelSt->bvec,*(lPedInView[i].currBelSt)->bvec);
	double max_value=-1;
	int max_index=-1;
	for(int iter=0;iter<length;iter++)
	{
		double v;
		v=(lPedInView[i].currBelSt)->bvec->data[iter].value;
		if(v>max_value) {max_value=v;max_index=iter;}
	}
	for(int iter=0;iter<length;iter++)
	{
		if(iter==max_index)
			(bmlBelSt->bvec)->data[iter].value =1.0;
		else
			(bmlBelSt->bvec)->data[iter].value =0.0;
	}
	cout<<"ML belief state"<<endl;
	(lPedInView[i].currBelSt)->bvec->write(cout);cout<<endl;
	(bmlBelSt->bvec)->write(cout);cout<<endl;
	bmlBelSt->sval=lPedInView[i].currBelSt->sval;

	Car car=world.GetCarPos();
	int lane=car.w;
	cout<<"Using Policy "<<lane<<endl;
	if(lane<0||lane>4) return 0;

	return qmdp_policies[lane]->getBestAction(*(bmlBelSt));
}

void ped_momdp::publishROSState()
{
	int w0,h0,w1,h1,w2,h2,w3,h3;
	w0=world.window.w0;
	h0=world.window.h0;
	w1=world.window.w1;
	h1=world.window.h1;
	w2=world.window.w2;
	h2=world.window.h2;
	w3=world.window.w3;
	h3=world.window.h3;
	double rln=ModelParams::rln;

	geometry_msgs::PolygonStamped plg;
	geometry_msgs::Point32 pnt;
	
	
	pnt.x=(w0+0.0)/rln;pnt.y=(h0+0.0)/rln;pnt.z=0;
	plg.polygon.points.push_back(pnt);
	pnt.x=(w1+0.0)/rln;pnt.y=(h1+0.0)/rln;pnt.z=0;
	plg.polygon.points.push_back(pnt);
	pnt.x=(w2+0.0)/rln;pnt.y=(h2+0.0)/rln;pnt.z=0;
	plg.polygon.points.push_back(pnt);
	pnt.x=(w3+0.0)/rln;pnt.y=(h3+0.0)/rln;pnt.z=0;
	plg.polygon.points.push_back(pnt);
	plg.header.stamp=ros::Time::now();
	plg.header.frame_id="map";
	//plg.points[0].x=w0; plg.points[0].y=h0; plg.points[0].z=0;
	//plg.points[1].x=w1; plg.points[1].y=h1; plg.points[1].z=0;
	//plg.points[2].x=w2; plg.points[2].y=h2; plg.points[2].z=0;
	//plg.points[3].x=w3; plg.points[3].y=h3; plg.points[3].z=0;
	

	geometry_msgs::Pose pose;
	
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.stamp=ros::Time::now();
	pose_stamped.header.frame_id="map";

	pose_stamped.pose.position.x=(world.car.w+0.0)/rln;
	pose_stamped.pose.position.y=(world.car.h+0.0)/rln;
	pose_stamped.pose.orientation.w=1.0;
	car_pub.publish(pose_stamped);
	
	geometry_msgs::PoseArray pA;
	pA.header.stamp=ros::Time::now();
	pA.header.frame_id="map";
	for(int i=0;i<world.ped_list.size();i++)
	{
		//GetCurrentState(ped_list[i]);
		pose.position.x=(world.ped_list[i].w+0.0)/rln;
		pose.position.y=(world.ped_list[i].h+0.0)/rln;
		pose.orientation.w=1.0;
		pA.poses.push_back(pose);
		//ped_pubs[i].publish(pose);
	}

	pa_pub.publish(pA);
	window_pub.publish(plg);	
	ros::Rate loop_rate(1);
	loop_rate.sleep();
}*/

/*
int N_Ped=3;
int np_map[4]={15,30,50};
void ped_momdp::simLoop()
{
	

	ofstream log_out("result.log");
	for(int outer=0;outer<=4;outer++)
	{
		for(int np=0;np<3;np++)
		{
		world.NumPedTotal=np_map[np];
		double total_reward=0;
		int j;
		int crush_count=0;
		int danger_count=0;
		double t=0;
		double t_sq=0;

		bool crushed=false;

		srand(unsigned(time(0)));
		world.SetSeed(rand()+37);
		for(j=0;j<4000;j++)
			//	for(j=0;j<1000;j++)
		{
			//world.SetSeed(rand());
			world.Init();
			initSimulator();
			crushed=false;
			ped_momdp_sarsop::ped_local_frame ped;
			double final_reward=0;
			double discount=1.0;

			int observation;
			double reward;
			int action;
			int safeAction;
			int step;
			for(step=0;step<2000;step++)
			{
				//action=reactivePolicy(state);

				safeAction=1;
				for(int i=0;i<lPedInView.size();i++)
				{
					if(outer==0)
						//lPedInView[i].currAction = policy->getBestActionLookAhead(*(lPedInView[i].currBelSt));
						lPedInView[i].currAction=getActionMOMDP(i);
					else if(outer==1)
						lPedInView[i].currAction = getActionQMDP(i);
					else if(outer==2)
						lPedInView[i].currAction=getActionBML(i);
					else if(outer==3)
						lPedInView[i].currAction=reactivePolicy(i);
					else if(outer==4)
						lPedInView[i].currAction = rand()%3;

					//cout<<"currAction "<<lPedInView[i].currAction<<endl;
					//getActionBML(i);
					//lPedInView[i].currAction=reactivePolicy(state_list[i]);

					if( (lPedInView[i].currAction!=safeAction) && (safeAction!=2) )
					{
						if(lPedInView[i].currAction==2)
							safeAction = 2;
						else if (lPedInView[i].currAction==0)
							safeAction =0;
					}		
				}
				cout<<"mySafeAction :"<<safeAction<<endl;
				//action=Executers[0]->GetAction();
				//Real->RobStep(*rob_state,safeAction;
				if(world.OneStep(safeAction)) break;
				//rob_state=safe_cast<PEDESTRIAN_DYNAMIC_REAL_STATE*>(rob_state);
				updatePedPoses();
				//shift window
				for(int i=0;i<lPedInView.size();i++)
				{
					updateBelief(i,safeAction);	
				}
				if(ModelParams::debug)
				{
					world.Display();
				}
				world.ShiftWindow();
				updatePedPoses();
				clean_momdp_problem_sim();
				updateObsStates();

				//publishROSState();

				//updatePomcpBelief(0,action);
			}
			if(world.InCollision(safeAction))  {
				crushed=true;
				if(world.InRealCollision(safeAction))
					crush_count++;
				danger_count++;
				cout<<"in danger "<<endl;
			}
			if(!crushed)  
			{
				t=t+step+1;
				t_sq+=(step+1)*(step+1);
			}

			lPedInView.clear();
		}
		
		log_out<<"Number of Pedestrians "<<np_map[np]<<endl;
		if(outer==0) log_out<<"MOMDP Result"<<endl;
		else if(outer==1) log_out<<"QMDP Result"<<endl;
		else if(outer==2) log_out<<"BML Result"<<endl;
		else if(outer==3) log_out<<"Reactive Result"<<endl;
		else if(outer==4) log_out<<"Random Result "<<endl;

		//cout<<"final reward : "<<final_reward<<endl;
		log_out<<"crush count:"<<crush_count<<endl;
		log_out<<"crush rate: "<<crush_count/(j+1+0.0)<<endl;
		log_out<<"danger count:"<<danger_count<<endl;
		log_out<<"danger rate: "<<danger_count/(j+1+0.0)<<endl;
		//cout<<"total reward "<<total_reward/(j+1)<<endl;
		log_out<<"total time "<<t/(j+1-crush_count)<<endl;
		log_out<<"time stderr "<<sqrt(((t_sq/(j+1))-((t/(j+1))*(t/(j+1))))/(j+1))<<endl;
		//if(crushed==true) break;
		cerr<<"outer "<<outer<<"np "<<np<<endl;
		}
	}
	cerr<<"loop finished"<<endl;
}*/

void ped_momdp::controlLoop(const ros::TimerEvent &e)
{
        if(RealWorldPt->ped_list.size()==0) return;   //no pedestrian detected yet
		RealWorldPt->ShiftWindow();
		RealSimulator->rob_map=RealWorldPt->window.rob_map;
		OBS_TYPE obs=RealWorldPt->GetCurrObs();

		PedestrianState ped_state=RealWorldPt->GetCurrState();
		RealSimulator->SetStartState(ped_state);

		double reward;
		solver->Update(safeAction, obs, reward, &ped_state);
	
		////////////////////////////////////////////////////////////////////////
		//
		safeAction=solver->SelectAction();
        if(safeAction==0) momdp_speed_ += 0;
        else if(safeAction==1) momdp_speed_ += 1.0;
        else if(safeAction==2) momdp_speed_ -= 1.0;
        if(momdp_speed_<=0) momdp_speed_ = 0;
        if(momdp_speed_>=2.0) momdp_speed_ = 2.0;

}
/*
int enter=false;
void ped_momdp::controlLoop(const ros::TimerEvent &e)
{

    cout << "Control loop dj .. " << " lPedInView " << lPedInView.size() << endl;

    //publish goal point at 25 meter in global frame
 

    if(lPedInView.size()==0)
    {
        momdp_speed_ = 0.0;
        return;
    }
    ///Start pomdp stuff
    cout << "=====================================================================" << endl;

    /// update belief based on the action taken
	
	if(enter==false)
	{
	  enter=true;
	}
	else
	{
		for(int ii=0; ii<lPedInView.size(); ii++)
		{
			int id = lPedInView[ii].id;
			updateBelief(id,safeAction);
			//updatePomcpBelief(ii,safeAction);
		}
	}

    //clean_momdp_problem();
    /// Get new action
    for(int ii=0; ii<lPedInView.size(); ii++)
    {
       lPedInView[ii].currAction =0;// policy->getBestActionLookAhead(*(lPedInView[ii].currBelSt));
       //lPedInView[ii].currAction =Executers[ii]->GetAction();
       cout<<"momdp action:"<<lPedInView[ii].currAction<<endl;
    }

    /// combining the actions by picking the safest action
    /// Do the far away pedestrians create a freezing action ???

    safeAction=1; /// actions : cru=0, acc=1, decc=2
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
        else if(safeAction==1) momdp_speed_ += 1.0;
        else if(safeAction==2) momdp_speed_ -= 1.0;
        if(momdp_speed_<=0) momdp_speed_ = 0;
        if(momdp_speed_>=2.0) momdp_speed_ = 2.0;
    }

	cout<<"momdp speed"<<momdp_speed_<<endl;
    publish_belief();


	//clean_momdp_problem();
}*/

/*
void ped_momdp::clean_momdp_problem_sim()
{
	for(int i=0;i<lPedInView.size();i++)
	{
		if(lPedInView[i].obss_updated==false)  	lPedInView.erase(lPedInView.begin()+i);
	}
}*/

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

    //policy = new AlphaVectorPolicy(problem);
	//qmdp_policy=new AlphaVectorPolicy(problem);

    cout << "\nLoading the policy ..." << endl;
    cout << "  input file   : " << solver_param->policyFile << endl;
    //bool policyRead = policy->readFromFile(solver_param->policyFile);
	bool policyRead=true;

	
	char buf[1000];	
	sprintf(buf,"%s",policy_file.c_str());
	cout<<"Reading QMDP policy file "<<endl;
	
	cout<<buf<<endl;
	
	
	buf[strlen(buf)-7]=0;

	//initialize the policy for each lane
	
	for(int i=0;i<5;i++)
	{
		char lane_buf[1000];
		sprintf(lane_buf,"%s_%d.policy",buf,i);
		policies[i]=new AlphaVectorPolicy(problem);
		policyRead=(policyRead&&(policies[i]->readFromFile(lane_buf)));	
		qmdp_policies[i]=new AlphaVectorPolicy(problem);
		sprintf(lane_buf,"%s_%d_qmdp.policy",buf,i);
		policyRead=(policyRead&&(qmdp_policies[i]->readFromFile(lane_buf)));	
	}

/*
	sprintf(buf,"%s_qmdp.policy",buf);
	cout<<buf<<endl;
	policyRead=(policyRead&&(qmdp_policy->readFromFile(buf)));	
*/
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
    engine.setup(problem, policies[2], solver_param); /// TBP : p is params

    double reward = 0, expReward = 0;

    /// Mapping states for quick reference
	cout<<"ObsStateMapping"<<endl;
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

void ped_momdp::updatePomcpBelief(int i, int safeAction)
{
    int px = getXGrid(lPedInView[i].ped_pose.x);
    int py = getYGrid(lPedInView[i].ped_pose.y);
    if(py==Y_SIZE-1) py=Y_SIZE-2;



    int ry = getYGrid(lPedInView[i].rob_pose);

    //if(myDebug)
    //sprintf(rob_str,"sR%02d",currRobY);

    cout<<"rob_pos"<<ry<<endl;
    if(ry>Y_SIZE-1)
    {
        ROS_WARN("Robot went outside of the grid size, pushing the robot pose back to the edge");
        ry = Y_SIZE-1;
    }


    //dj: discretization of robot speed into 3 int levels (0,1,2)
    double rvel_double;
    double currRobSpeed=robotspeedx_;
    if(currRobSpeed < 0.1) rvel_double = 0;
    else if(currRobSpeed > 0.1 && currRobSpeed < 2) rvel_double = 1;
    else if(currRobSpeed >= 2) rvel_double = 2;
    //double rvel_double =  currRobSpeed/1.5*2;//dSpeed;
	//int rvel=(int) momdp_speed_;
    int rvel= (int) rvel_double;
	cout<<"currRobSpeed:"<<currRobSpeed<<endl;
    cout<<"rvel:"<<rvel<<endl;
	cout<<"px py: "<<px<<" "<<py<<endl;
	cout<<"ry: "<<ry<<endl;
	cout<<"safeAction: "<<safeAction<<endl;
    //if(rvel_double < 0.0001 )
    int observation=rvel*500+ry*41+px*11+py;
	 //Executers[i]->Step(safeAction,observation);
}
void ped_momdp::updateBelief(int i, int safeAction)
{

    //int nextSVal = getCurrentState(robotspeedx_, lPedInView[ii].rob_pose, lPedInView[ii].ped_pose.x, lPedInView[ii].ped_pose.y);

    /// Debug
    int nextSVal = getCurrentState(robotspeedx_,  lPedInView[i].rob_pose, lPedInView[i].ped_pose.x, lPedInView[i].ped_pose.y,lPedInView[i].id);
    int currObservation = getCurrObs(i);
	cout<<"momdp observation "<<currObservation<<endl;

    cout << "before update belief" << endl;
    (lPedInView[i].currBelSt)->bvec->write(cout); cout << endl;
    cout << "curr bel sval " << (lPedInView[i].currBelSt)->sval << endl;

    SharedPointer<BeliefWithState> nextBelSt;
    engine.runStep((lPedInView[i].currBelSt), safeAction, currObservation, nextSVal, nextBelSt );

    copy(*(lPedInView[i].currBelSt)->bvec, *nextBelSt->bvec);
    (lPedInView[i].currBelSt)->sval = nextSVal;

    cout << "next belief" << endl;
    (lPedInView[i].currBelSt)->bvec->write(cout); cout << endl;
    cout << "next bel sval " << (lPedInView[i].currBelSt)->sval << endl;
	
	if((lPedInView[i].currBelSt)->bvec->data.size()==0) 
	{    
		cout<<"Observation unmatch, restart the belief vector "<<endl;
		SharedPointer<SparseVector> startBeliefVec;
		if (problem->initialBeliefStval->bvec)
			startBeliefVec = problem->initialBeliefStval->bvec;
		else
			startBeliefVec = problem->initialBeliefYByX[lPedInView[i].currSVal];

		copy(*(lPedInView[i].currBelSt)->bvec, *startBeliefVec);
		(lPedInView[i].currBelSt)->bvec->write(cout); cout << endl;

	}

    //map<string, string> aa = problem->getActionsSymbols(safeAction);
    //cout << "safe action " << aa["action_robot"] << endl;

    ROS_INFO ("next bel sval %d", (lPedInView[i].currBelSt)->sval);
	lPedInView[i].obss_updated=false;
}

int ped_momdp::getCurrentState(double currRobSpeed, double roby, double pedx, double pedy,int ped_id)
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
    else if(currRobSpeed >= 2) rvel_double = 2;
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

    cout << " Curr state " << state_str << " id " << ped_id << endl;

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

    /// Bin continuous values to get discrupdateBeliefete values
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
