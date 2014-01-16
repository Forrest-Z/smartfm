#include "momdp.h"
#include "belief_update/belief_update_particle.h"
#include "vnode.h"
#include "solver.h"
#include "problems/pedestrian_changelane/pedestrian_changelane_despot.h"

Model<PedestrianState>* RealSimulator;
Solver<PedestrianState>* solver;
ILowerBound<PedestrianState>* lb;
BeliefUpdate<PedestrianState>* bu;
RandomStreams streams(Globals::config.n_particles, Globals::config.search_depth, 
                        Globals::config.root_seed);

int WorldSeed() {
  cout<<"root seed"<<Globals::config.root_seed<<endl;
  return Globals::config.root_seed ^ Globals::config.n_particles;
}

int BeliefUpdateSeed() {
  return Globals::config.root_seed ^ (Globals::config.n_particles + 1);
}

// Action selection for RandomPolicyLowerBound (if used):
int RandomActionSeed() {
  return Globals::config.root_seed ^ (Globals::config.n_particles + 2);
}

ped_momdp::ped_momdp(string model_file, string policy_file, int simLen, int simNum, bool stationary, double frequency, bool use_sim_time, ros::NodeHandle& nh,WorldSimulator*rw)
{
	cout<<"momdp node start"<<endl;
    X_SIZE=4;
	//X_SIZE=ModelParams::XSIZE;
    Y_SIZE=ModelParams::YSIZE;
    dY= 1;// step size in Y
    dX= 1;// step size in X
	control_freq=frequency;

    momdp_problem_timeout = 5.0;
    stationary_ = stationary;
    use_sim_time_ = use_sim_time;
    believesPub_ = nh.advertise<ped_momdp_sarsop::peds_believes>("peds_believes",1);
    cmdPub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
	actionPub_ = nh.advertise<visualization_msgs::Marker>("pomdp_action",1);
	char buf[100];
	for(int i=0;i<ModelParams::N_PED_IN;i++) {
		sprintf(buf,"pomdp_beliefs%d",i);
		markers_pubs[i]=nh.advertise<visualization_msgs::MarkerArray>(buf,1);
	}
    //initPedGoal();
    //policy_initialize(model_file, policy_file, simLen, simNum);
	safeAction=2;
	momdp_speed_=0.0;
	RealWorldPt=rw;
	initRealSimulator();
	cout<<"before entering controlloop"<<endl;
	cout<<"frequency "<<frequency<<endl;
  	timer_ = nh.createTimer(ros::Duration(1.0/frequency), &ped_momdp::controlLoop, this);
	timer_speed=nh.createTimer(ros::Duration(0.1), &ped_momdp::publishSpeed, this);



	cout<<"here"<<endl;
	//initRealSimulator();

}



/*for pomcp
void ped_momdp::initRealSimulator()
{

	RealSimulator=new PEDESTRIAN_CHANGELANE(ModelParams::XSIZE,ModelParams::YSIZE);
	RealSimulator->rob_map=RealWorldPt->window.rob_map;
	RealSimulator->sfm=&RealWorldPt->sfm;

	PedestrianState startState=RealWorldPt->GetCurrState();
	RealSimulator->SetStartState(startState);
	cout<<"Initial State"<<endl;
	RealSimulator->DisplayState(startState,cout);

    MCTS::InitFastUCB(50000);
    MCTS::PARAMS SearchParams;
	solver=new MCTS(*RealSimulator, SearchParams);
	solver->root_list.push_back(solver->Root);

	solver->pedproblem_c=RealSimulator;

	//cout<<"Initial Belief"<<endl;
	//RealSimulator->DisplayBeliefs(solver->Root->Beliefs(),cout);


}*/
bool ped_momdp::getObjectPose(string target_frame, tf::Stamped<tf::Pose>& in_pose, tf::Stamped<tf::Pose>& out_pose) const
{
    out_pose.setIdentity();

    try {
        tf_.transformPose(target_frame, in_pose, out_pose);
    }
    catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        return false;
    }
    return true;
}


/*for despot*/
void ped_momdp::initRealSimulator()
{
  Globals::config.root_seed=1024;
  cout<<"global particle "<<Globals::config.n_particles<<endl;
  cout<<"root seed "<<Globals::config.root_seed<<endl;
  cout<<"search depth"<<Globals::config.search_depth<<endl;


  RealSimulator  = new Model<PedestrianState>(streams, "pedestrian.config");




  int knowledge = 2;
  lb = new RandomPolicyLowerBound<PedestrianState>(
		  streams, knowledge, RandomActionSeed());

  cout<<"lower bound address "<<lb<<endl;
  //IUpperBound<PedestrianState>* ub =
      //new UpperBoundStochastic<PedestrianState>(streams, *model);

  bu = new ParticleFilterUpdate<PedestrianState>(BeliefUpdateSeed(), *RealSimulator);

  // int ret = Run(model, lb, ub, bu, streams);
  VNode<PedestrianState>::set_model(*RealSimulator);
  RealSimulator->rob_map=RealWorldPt->window.rob_map;
  RealSimulator->sfm=&RealWorldPt->sfm;
  int action;
  int i;

  PedestrianState ped_state=RealWorldPt->GetCurrState();

  cout<<"start state"<<endl;
  RealSimulator->PrintState(ped_state);

  RealSimulator->SetStartState(ped_state);
  solver=new Solver<PedestrianState>(*RealSimulator, RealSimulator->InitialBelief(), *lb, *RealSimulator, *bu, streams);	
  solver->Init();
  int n_trials;
  solver->Search(1.0/control_freq,n_trials);

}



ped_momdp::~ped_momdp()
{
    geometry_msgs::Twist cmd;
    cmd.angular.z = 0;
    cmd.linear.x = 0;
    cmdPub_.publish(cmd);
}



void ped_momdp::updateSteerAnglePublishSpeed(geometry_msgs::Twist speed)
{
    //cmdPub_.publish(speed);
}

void ped_momdp::publishSpeed(const ros::TimerEvent &e)
{
	geometry_msgs::Twist cmd;
	cmd.angular.z = 0;       	
	cmd.linear.x = momdp_speed_;
	cout<<"publishing cmd speed "<<momdp_speed_<<endl;
	cmdPub_.publish(cmd);
}



void ped_momdp::publishROSState()
{
	int w0,h0,w1,h1,w2,h2,w3,h3;
	w0=RealWorldPt->window.w0;
	h0=RealWorldPt->window.h0;
	w1=RealWorldPt->window.w1;
	h1=RealWorldPt->window.h1;
	w2=RealWorldPt->window.w2;
	h2=RealWorldPt->window.h2;
	w3=RealWorldPt->window.w3;
	h3=RealWorldPt->window.h3;
	double rln=ModelParams::map_rln;

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
	plg.header.frame_id="/golfcart/map";
	//plg.points[0].x=w0; plg.points[0].y=h0; plg.points[0].z=0;
	//plg.points[1].x=w1; plg.points[1].y=h1; plg.points[1].z=0;
	//plg.points[2].x=w2; plg.points[2].y=h2; plg.points[2].z=0;
	//plg.points[3].x=w3; plg.points[3].y=h3; plg.points[3].z=0;
	

	geometry_msgs::Pose pose;
	
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.stamp=ros::Time::now();
	pose_stamped.header.frame_id="/golfcart/map";

	pose_stamped.pose.position.x=(RealWorldPt->car.w+0.0)/rln;
	pose_stamped.pose.position.y=(RealWorldPt->car.h+0.0)/rln;
	pose_stamped.pose.orientation.w=1.0;
	car_pub.publish(pose_stamped);
	
	geometry_msgs::PoseArray pA;
	pA.header.stamp=ros::Time::now();
	pA.header.frame_id="/golfcart/map";
	for(int i=0;i<RealWorldPt->ped_list.size();i++)
	{
		//GetCurrentState(ped_list[i]);
		pose.position.x=(RealWorldPt->ped_list[i].w+0.0)/rln;
		pose.position.y=(RealWorldPt->ped_list[i].h+0.0)/rln;
		pose.orientation.w=1.0;
		pA.poses.push_back(pose);
		//ped_pubs[i].publish(pose);
	}

	pa_pub.publish(pA);
	window_pub.publish(plg);	
	ros::Rate loop_rate(1);
	//loop_rate.sleep();
}


//for pomcp
/*
void ped_momdp::controlLoop(const ros::TimerEvent &e)
{
	    cout<<"entering control loop"<<endl;
		RealWorldPt->ShiftWindow();
		publishROSState();
		cout<<"here"<<endl;
        if(RealWorldPt->NumPedInView()==0) return;   //no pedestrian detected yet
		RealSimulator->rob_map=RealWorldPt->window.rob_map;
		OBS_T obs=RealWorldPt->GetCurrObs();

		cout<<"world observation "<<obs<<endl;
		PedestrianState ped_state=RealWorldPt->GetCurrState();
		RealSimulator->SetStartState(ped_state);
		RealSimulator->DisplayState(ped_state,cout);

		double reward;
		solver->Update(safeAction, obs, reward, &ped_state);
	
		////////////////////////////////////////////////////////////////////
		
		safeAction=solver->SelectAction();
		
		cout<<"safe action "<<safeAction<<endl;

		
        if(safeAction==0) momdp_speed_ += 0;
        else if(safeAction==1) momdp_speed_ += 1.0;
        else if(safeAction==2) momdp_speed_ -= 1.0;
        if(momdp_speed_<=0) momdp_speed_ = 0;
        if(momdp_speed_>=2.0) momdp_speed_ = 2.0;
		

		cout<<"momdp_spped "<<momdp_speed_<<endl;
		publishBelief();

}*/

double marker_colors[20][3] = {
	{0.0,1.0,0.0},
		{1.0,0.0,0.0},
		{0.0,0.0,1.0},
		{1.0,1.0,0.0},
		{0.0,1.0,1.0},
		{1.0,0.0,1.0},
		{1.0,1.0,1.0}
};

int action_map[3]={2,0,1};

void ped_momdp::publishAction(int action)
{
		uint32_t shape = visualization_msgs::Marker::CUBE;
		visualization_msgs::Marker marker;			
		marker.header.frame_id="/golfcart/map";
		marker.header.stamp=ros::Time::now();
		marker.ns="basic_shapes";
		marker.id=0;
		marker.type=shape;
		marker.action = visualization_msgs::Marker::ADD;

		double px,py;
		px=RealWorldPt->car.w/ModelParams::map_rln;
		py=RealWorldPt->car.h/ModelParams::map_rln;
		marker.pose.position.x = px+1;
		marker.pose.position.y = py+1;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.6;
		marker.scale.y = 3;
		marker.scale.z = 0.6;
		//
		// Set the color -- be sure to set alpha to something non-zero!
		//marker.color.r = 0.0f;
		//marker.color.g = 1.0f;
		//marker.color.b = 0.0f;
		//marker.color.a = 1.0;
		marker.color.r = marker_colors[action_map[action]][0];
        marker.color.g = marker_colors[action_map[action]][1];
		marker.color.b = marker_colors[action_map[action]][2];//marker.lifetime = ros::Duration();
		marker.color.a = 1.0;

		ros::Duration d(1/control_freq);
		marker.lifetime=d;
		actionPub_.publish(marker);		

}
//for despot
void ped_momdp::controlLoop(const ros::TimerEvent &e)
{
	    cout<<"entering control loop"<<endl;
			

        tf::Stamped<tf::Pose> in_pose, out_pose;
		in_pose.setIdentity();
		in_pose.frame_id_ = "/golfcart/base_link";
		if(!getObjectPose("/golfcart/map", in_pose, out_pose)) {
			cerr<<"transform error within control loop"<<endl;
		} else {
			Car world_car;
			world_car.w=out_pose.getOrigin().getX()*ModelParams::map_rln;
			world_car.h=out_pose.getOrigin().getY()*ModelParams::map_rln;
			RealWorldPt->UpdateRobPoseReal(world_car);
		}
		
		publishROSState();

		if(RealWorldPt->GoalReached())
		{
			safeAction=2;

			momdp_speed_=real_speed_;
			if(safeAction==0) momdp_speed_ += 0;
			else if(safeAction==1) momdp_speed_ += 0.5;
			else if(safeAction==2) momdp_speed_ -= 0.5;
			if(momdp_speed_<=0.1) momdp_speed_ = 0.1;
			if(momdp_speed_>=2.0) momdp_speed_ = 2.0;

			return;
		}



		cout<<"State before shift window"<<endl;
		RealSimulator->PrintState(RealWorldPt->GetCurrState());
		RealWorldPt->ShiftWindow();
		cout<<"here"<<endl;
        //if(RealWorldPt->NumPedInView()==0) return;   //no pedestrian detected yet
		RealSimulator->rob_map=RealWorldPt->window.rob_map;
		OBS_T obs=RealWorldPt->GetCurrObs();

		cout<<"world observation "<<obs<<endl;
		PedestrianState ped_state=RealWorldPt->GetCurrState();
		cout<<"current state"<<endl;
		RealSimulator->PrintState(ped_state);

		double reward;
		solver->UpdateBelief(safeAction, obs,  ped_state);
	
		////////////////////////////////////////////////////////////////////
		
		int n_trials;
		cout<<"move time "<<Globals::config.time_per_move<<endl;

		safeAction=solver->Search(1.0/control_freq,n_trials);

		//actionPub_.publish(action);
		publishAction(safeAction);

		cout<<"n trials "<<n_trials<<endl;
		
		cout<<"safe action "<<safeAction<<endl;

		
		

		momdp_speed_=real_speed_;
        if(safeAction==0) momdp_speed_ += 0;
        else if(safeAction==1) {
			if(momdp_speed_ < 0.6) {
				momdp_speed_ = 0.7;
			} else {
				momdp_speed_ += 0.5;
			}
		}
        else if(safeAction==2) momdp_speed_ -= 0.5;
        if(momdp_speed_<=0.1) momdp_speed_ = 0.1;
        if(momdp_speed_>=2.0) momdp_speed_ = 2.0;
		
		

		cout<<"momdp_spped "<<momdp_speed_<<endl;

		publishBelief();

}


void ped_momdp::publishMarker(int id,vector<double> belief)
{
	visualization_msgs::MarkerArray markers;
	uint32_t shape = visualization_msgs::Marker::CUBE;

	for(int i=0;i<belief.size();i++)
	{
		visualization_msgs::Marker marker;			
		marker.header.frame_id="/golfcart/map";
		marker.header.stamp=ros::Time::now();
		marker.ns="basic_shapes";
		marker.id=i;
		marker.type=shape;
		marker.action = visualization_msgs::Marker::ADD;

		double px,py;
		px=RealWorldPt->ped_list[RealWorldPt->pedInView_list[id]].w/ModelParams::map_rln;
		py=RealWorldPt->ped_list[RealWorldPt->pedInView_list[id]].h/ModelParams::map_rln;
		marker.pose.position.x = px+i*0.7;
		marker.pose.position.y = py+belief[i]*2;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		cout<<"belief entries "<<px<<" "<<py<<endl;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.5;
		marker.scale.y = belief[i]*4;
		if(marker.scale.y<0.2) marker.scale.y=0.2;
		marker.scale.z = 0.2;
		//
		// Set the color -- be sure to set alpha to something non-zero!
		//marker.color.r = 0.0f;
		//marker.color.g = 1.0f;
		//marker.color.b = 0.0f;
		//marker.color.a = 1.0;
		marker.color.r = marker_colors[i][0];
        marker.color.g = marker_colors[i][1];
		marker.color.b = marker_colors[i][2];//marker.lifetime = ros::Duration();
		marker.color.a = 1.0;

		ros::Duration d(1/control_freq);
		marker.lifetime=d;
		markers.markers.push_back(marker);
	}
	markers_pubs[id].publish(markers);

}
void ped_momdp::publishBelief()
{
	vector<vector<double> > ped_beliefs=RealSimulator->GetBeliefVector(solver->root_->particles());	
	cout<<"belief vector size "<<ped_beliefs.size()<<endl;
	for(int i=0;i<ped_beliefs.size();i++)
	{
		publishMarker(i,ped_beliefs[i]);
	}

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



